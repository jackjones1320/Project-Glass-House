"""Live UDP detector entry point.

Receives IQ packets from the coordinator's SoftAP forwarder, computes
per-link breathing-band Doppler signatures over a rolling 90-second
window, and emits a per-window quadrant prediction via the continuous-
attribution estimator + HMM smoother in `live.continuous_estimator`.

Pipeline
--------
At each emit tick (default every 1 s):
   1. Take the last 90 s of IQ records and group by unordered link
      (i, j) in {(1,2), ..., (3,4)}.
   2. For each link, build an (n_packets, n_subcarriers) magnitude
      matrix; optionally apply CARM-style SVD drift removal.
   3. Run Welch's method per subcarrier on the breathing band
      (0.15-0.40 Hz) to extract per-link active_fraction and peak SNR.
   4. Run the continuous-attribution node estimator: each link
      contributes 0.5 * signal_score to its two endpoint nodes,
      reliability-weighted by sqrt(n_packets / max_n_packets).
   5. Update the 4-state HMM forward filter; the argmax of the
      posterior is the displayed quadrant.

Geometry (4-corner room layout):
  Q1 = NW = node 2,  Q2 = NE = node 3,
  Q3 = SW = node 1,  Q4 = SE = node 4.

Usage
-----
    # Live UDP from coord SoftAP forwarder (no baseline needed)
    python -m live.deterministic_detector udp \
        --continuous-estimator --svd-drift-modes 1 \
        --localize --pygame --pygame-fullscreen
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Dict, Iterator, Tuple

import numpy as np

_HERE = Path(__file__).resolve()
_PYDIR = _HERE.parent.parent
if str(_PYDIR) not in sys.path:
    sys.path.insert(0, str(_PYDIR))

def _parse_iq_complex(record: dict) -> np.ndarray:
    """Decode an IQ JSONL record into a complex64 array.

    Mirrors the firmware-side IQ wire format. Two header layouts are
    supported (selected by ``wire_version``): v1 has an 8-byte header,
    v2 has a 10-byte header. After the header, the payload is
    ``iq_len`` bytes of interleaved int8 I/Q samples; pair them and
    cast to complex64.
    """
    iq_len = record.get("iq_len", 0)
    wire_version = record.get("wire_version", 1)
    header_size = 10 if wire_version == 2 else 8
    raw = record.get("raw", "")
    if not raw:
        return np.array([], dtype=np.complex64)
    full = bytes.fromhex(raw)
    if len(full) < header_size + iq_len:
        return np.array([], dtype=np.complex64)
    payload = full[header_size:header_size + iq_len]
    n_sc = len(payload) // 2
    if n_sc == 0:
        return np.array([], dtype=np.complex64)
    iq = np.frombuffer(payload[:n_sc * 2], dtype=np.int8)
    return (
        iq[0::2].astype(np.float32) + 1j * iq[1::2].astype(np.float32)
    ).astype(np.complex64)
from live import localizer_2d  # noqa: E402
from live.continuous_estimator import (  # noqa: E402
    continuous_node_estimator,
    HmmNodeSmoother,
    blockage_attribution,
)


# Canonical 4-node link list (sorted, lower-id first).
ALL_LINKS = [(1, 2), (1, 3), (1, 4), (2, 3), (2, 4), (3, 4)]


def _svd_denoise_amps(amps, n_drift_modes=1, log_amplitude=True, eps=1e-9):
    """Remove top-K temporal modes via SVD (CARM-style drift cancellation).

    Args:
        amps: (n_pkts, n_sc) per-packet amplitude matrix for one link.
        n_drift_modes: how many leading singular vectors to discard.
            K=1 removes the dominant temporal mode (slow drift).
            K=2 removes drift + the next-strongest common mode.
        log_amplitude: operate in log10 space so multiplicative drift
            becomes additive. Reverts to linear after denoise.
        eps: clipping floor before log.

    Returns the denoised (n_pkts, n_sc) matrix in linear amplitude.

    Math: X (n_pkts x n_sc), columns centered. SVD gives X = U S Vt.
    The first column of U (a temporal pattern) and first row of Vt (a
    subcarrier pattern) define the strongest temporal-spatial mode.
    Slowly varying drift across all subcarriers concentrates in this
    mode; dropping it leaves only body-signature variation orthogonal
    to drift. Apply only on windows long enough that drift dominates
    breathing in the dominant mode (>= 30 s).
    """
    if amps.shape[0] < 4:
        return amps
    if log_amplitude:
        amps_op = np.log10(np.maximum(amps, eps))
    else:
        amps_op = amps.astype(np.float64, copy=True)
    col_mean = amps_op.mean(axis=0, keepdims=True)
    X = amps_op - col_mean
    try:
        U, S, Vt = np.linalg.svd(X, full_matrices=False)
    except np.linalg.LinAlgError:
        return amps
    keep = max(0, len(S) - max(0, n_drift_modes))
    if keep <= 0 or n_drift_modes >= len(S):
        return amps
    X_clean = (U[:, n_drift_modes:]
                * S[n_drift_modes:][None, :]) @ Vt[n_drift_modes:, :]
    amps_clean = X_clean + col_mean
    if log_amplitude:
        return np.power(10.0, amps_clean)
    return amps_clean


# Pipeline tunables. None of these are fitted parameters; they are
# hand-chosen safety margins.
WINDOW_SECONDS = 6.0
ADVANCE_SECONDS = 1.0
MAD_K = 3.0                  # outlier rejection threshold (Hampel-style)
RESET_GUARD_SECONDS = 5.0    # +/- around any node reboot in the live stream

# ESP-IDF esp_reset_reason_t enum -> human label. Keep this in sync
# with components/esp_system/include/esp_system.h. Unknown codes fall
# through to f"code {n}" so future IDF versions don't crash the
# display.
ESP_RESET_REASON_LABEL = {
    0:  "Unknown",
    1:  "Power-on",
    2:  "External pin",
    3:  "esp_restart()",
    4:  "Panic",
    5:  "Int WDT",
    6:  "Task WDT",
    7:  "Other WDT",
    8:  "Deep sleep",
    9:  "Brownout",
    10: "SDIO",
    11: "USB",
    12: "JTAG",
    13: "eFuse",
    14: "Power glitch",
    15: "CPU lockup",
}


def reset_reason_label(code):
    if code is None:
        return None
    return ESP_RESET_REASON_LABEL.get(int(code), f"code {int(code)}")

# Breathing-band Doppler detection physics:
# - Adult resting breathing: 12-20 breaths/min = 0.20-0.33 Hz
# - Athletes resting: 0.10-0.17 Hz; stressed up to ~0.50 Hz
# - Static multipath and drift produce 0 Hz content only; breathing
#   is the only mechanism creating structured power in this band.
# - 6 s window gives FFT resolution 0.167 Hz; breathing band spans
#   only 1-2 FFT bins (very low resolution).
# - 90 s window gives FFT resolution 0.011 Hz; breathing band spans
#   ~24 FFT bins. White-noise floor drops, breathing peak concentrates
#   in 1-2 bins of those 24 -> sharp peak-to-noise contrast.
BREATHING_BAND_HZ = (0.15, 0.40)
# Default Doppler buffer length. The variance gate keeps the short 6s
# window; Doppler uses its own rolling buffer for higher SNR.
BREATHING_WINDOW_SECONDS = 90.0
# Minimum buffer duration before Doppler scoring is meaningful. During
# the warm-up after a fresh start, the buffer fills from 0 to 90 s; we
# only score once at least 30 s of samples are available so the FFT has
# enough breath cycles (~10) to resolve the peak.
BREATHING_MIN_DURATION_S = 30.0
# Polynomial detrend order applied per-subcarrier before FFT. Order=2
# removes both DC and quadratic thermal drift over a 90 s window without
# eating into the 0.15 Hz breathing-band lower edge.
DETREND_ORDER = 2
# Threshold: fraction of spectral power that must lie in the breathing
# band (vs total non-DC power) to count a subcarrier as "showing
# breathing modulation". White noise distributes power uniformly, so
# the noise-floor for our 0.25 Hz band over a 0-10 Hz spectrum is
# ~0.025. Threshold 0.10 = 4x noise floor = body presence.
BREATHING_BAND_RATIO_THRESHOLD = 0.10
# Presence threshold: fraction of (link, subcarrier) pairs that must
# exceed BREATHING_BAND_RATIO_THRESHOLD for the room to be "occupied".
# Literature: 5-30% of subcarriers typically show clear breathing
# modulation (the ones whose multipath traverses the body). Threshold
# 0.05 = 5% of 1152 (link, sc) pairs = 58 pairs, well above noise.
BREATHING_PRESENCE_THRESHOLD = 0.05


def _link_key(node_id, peer_id):
    if node_id is None or peer_id is None or node_id == peer_id:
        return None
    a, b = sorted((int(node_id), int(peer_id)))
    if a < 1 or b > 4:
        return None
    return (a, b)


# --------------------------------------------------------------------------
# Per-(link, subcarrier) amplitude computation with Hampel-style cleaning
# --------------------------------------------------------------------------
def per_link_subcar_amps(records, mad_k=MAD_K):
    """Aggregate IQ records into per-link per-subcarrier mean amplitude
    with Hampel/MAD outlier rejection per subcarrier.

    `records` should be IQ-typed; non-IQ records are ignored. Returns
    {link: (n_sc,) ndarray} for links that had any contributing records.
    """
    per_link_seq = defaultdict(list)  # link -> list of (n_sc,) magnitudes
    for r in records:
        if r.get("type") != "iq":
            continue
        link = _link_key(r.get("node_id"), r.get("peer_id"))
        if link is None:
            continue
        csi = _parse_iq_complex(r)
        if csi.size == 0:
            continue
        per_link_seq[link].append(np.abs(csi).astype(np.float64))

    out = {}
    for link, packets in per_link_seq.items():
        if not packets:
            continue
        # Use the modal subcarrier count (most are 192 for HT40; HT20 fallback is 64).
        lens = [p.shape[0] for p in packets]
        common_len = int(np.median(lens))
        pkts = [p[:common_len] for p in packets if p.shape[0] >= common_len]
        if not pkts:
            continue
        M = np.stack(pkts, axis=0)  # (n_packets, n_sc)
        med = np.median(M, axis=0)
        mad = np.median(np.abs(M - med), axis=0)
        sigma = 1.4826 * mad
        thresh = mad_k * sigma
        keep = np.abs(M - med) <= thresh
        keep = np.where(sigma[None, :] > 0, keep, True)  # constant subcarrier -> keep all
        kept_sums = (M * keep).sum(axis=0)
        kept_cnts = keep.sum(axis=0).clip(min=1)
        out[link] = kept_sums / kept_cnts
    return out


def per_link_delta(window_records, baseline):
    """Compute per-(link, subcarrier) delta = current_mean - baseline."""
    cur = per_link_subcar_amps(window_records)
    deltas = {}
    for link, base_v in baseline.items():
        if link not in cur:
            continue
        if cur[link].shape != base_v.shape:
            continue
        deltas[link] = cur[link] - base_v
    return deltas


def per_link_doppler_power(window_records,
                           breathing_band=BREATHING_BAND_HZ,
                           min_packets=8,
                           min_duration_s=1.0,
                           detrend_order=DETREND_ORDER,
                           hampel_k=MAD_K,
                           use_hanning=True):
    """Compute per-(link, subcarrier) breathing-band spectral power ratio.

    For each (link, subcarrier), build the time series of |IQ| within
    this window, FFT it, and compute the fraction of spectral power
    that falls in the breathing band (default 0.15-0.40 Hz). Returns
    dict[link] -> (n_subcarriers,) ndarray of breathing-band ratios
    in [0, 1].

    Physics rationale:
    - Static multipath + drift produce 0 Hz spectral content only.
    - Breathing modulates body cross-section -> multipath length -> CSI
      amplitude with a periodic signal at 0.20-0.33 Hz typically.
    - White-noise floor at 6 s window: ratio ~0.05 (band spans 2-3 of
      ~60 bins). At 90 s window: ~0.027 (24 of ~899 bins). Longer
      windows give a much sharper signal-vs-noise discrimination.

    Pipeline per link:
      1. Stack |IQ| samples in time order -> (n_packets, n_sc) matrix.
      2. Hampel/MAD outlier filter ACROSS TIME per subcarrier: replace
         outliers with the per-subcarrier median (preserve uniform
         sampling required by FFT; dropping samples would create gaps).
      3. Polynomial detrend (default order=2): remove DC + linear +
         quadratic drift per subcarrier so thermal/gain trends don't
         leak power into the 0.15 Hz lower edge of the breathing band.
      4. Hanning window: ~30 dB sidelobe reduction so any residual DC
         leakage stays out of the breathing band.
      5. rFFT per subcarrier; power = |FFT|^2.
      6. Breathing-band ratio = sum(power in band) / sum(power non-DC).

    Returns {} if a link has fewer than min_packets or shorter duration
    than min_duration_s.
    """
    per_link_samples = defaultdict(list)
    for r in window_records:
        if r.get("type") != "iq":
            continue
        link = _link_key(r.get("node_id"), r.get("peer_id"))
        if link is None:
            continue
        csi = _parse_iq_complex(r)
        if csi.size == 0:
            continue
        t = float(r.get("t") if r.get("t") is not None else r.get("_t_rel", 0.0))
        per_link_samples[link].append((t, np.abs(csi).astype(np.float64)))

    out = {}
    for link, samples in per_link_samples.items():
        if len(samples) < min_packets:
            continue
        samples.sort(key=lambda s: s[0])
        ts = np.array([s[0] for s in samples])
        # Use the modal subcarrier count to avoid HT20/HT40 mixing.
        lens = [s[1].shape[0] for s in samples]
        common_len = int(np.median(lens))
        amps_list = [s[1][:common_len] for s in samples
                     if s[1].shape[0] >= common_len]
        if len(amps_list) < min_packets:
            continue
        amps = np.stack(amps_list, axis=0)  # (n_packets, n_sc)
        n_pkts, n_sc = amps.shape

        duration = ts[-1] - ts[0]
        if duration < min_duration_s:
            continue
        fs = (n_pkts - 1) / duration  # effective sample rate (Hz)

        # Step 2: Hampel filter across time per subcarrier. Replace
        # outliers with per-subcarrier median to keep the time grid
        # uniform for FFT.
        med_t = np.median(amps, axis=0)
        mad_t = np.median(np.abs(amps - med_t[None, :]), axis=0)
        sigma_t = 1.4826 * mad_t
        deviation = np.abs(amps - med_t[None, :])
        outliers = (sigma_t[None, :] > 0) & (deviation > hampel_k * sigma_t[None, :])
        if outliers.any():
            amps = np.where(outliers,
                            np.broadcast_to(med_t[None, :], amps.shape),
                            amps)

        # Step 3: polynomial detrend per subcarrier.
        # Build basis [1, x, x^2, ...] on x in [-1, 1] for numerical
        # conditioning; solve least-squares per subcarrier, subtract.
        if detrend_order >= 1:
            x = np.linspace(-1.0, 1.0, n_pkts).astype(np.float64)
            basis = np.vstack([x ** k for k in range(detrend_order + 1)]).T
            try:
                coeffs, *_ = np.linalg.lstsq(basis, amps, rcond=None)
                trend = basis @ coeffs
                amps_dt = amps - trend
            except np.linalg.LinAlgError:
                amps_dt = amps - amps.mean(axis=0)[None, :]
        else:
            amps_dt = amps - amps.mean(axis=0)[None, :]

        # Step 4: Hanning window (sidelobe rejection ~30 dB).
        if use_hanning and n_pkts >= 4:
            win = np.hanning(n_pkts).astype(np.float64).reshape(-1, 1)
            amps_w = amps_dt * win
        else:
            amps_w = amps_dt

        # Step 5+6: FFT, power spectrum, band ratio.
        spectrum = np.abs(np.fft.rfft(amps_w, axis=0))  # (n_freqs, n_sc)
        freqs = np.fft.rfftfreq(n_pkts, 1.0 / fs)

        in_band = (freqs >= breathing_band[0]) & (freqs <= breathing_band[1])
        all_band = freqs > 0  # exclude DC
        breathing_power = (spectrum[in_band] ** 2).sum(axis=0)
        total_power = (spectrum[all_band] ** 2).sum(axis=0)

        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(total_power > 0,
                              breathing_power / total_power, 0.0)
        out[link] = ratio
    return out


def breathing_presence_score(per_link_doppler,
                              ratio_threshold=BREATHING_BAND_RATIO_THRESHOLD):
    """Fraction of (link, subcarrier) pairs whose breathing-band ratio
    exceeds the threshold. Empty rooms score near zero; a breathing body
    in any quadrant typically scores 0.05-0.30."""
    total = 0
    above = 0
    for arr in per_link_doppler.values():
        total += int(arr.size)
        above += int((arr > ratio_threshold).sum())
    if total == 0:
        return 0.0
    return float(above) / float(total)


def doppler_self_explain(window_records,
                          breathing_band=BREATHING_BAND_HZ,
                          ratio_threshold=BREATHING_BAND_RATIO_THRESHOLD,
                          min_packets=8,
                          min_duration_s=BREATHING_MIN_DURATION_S,
                          detrend_order=DETREND_ORDER):
    """Compute per-link Doppler self-explanation features.

    Returns a dict suitable for human-readable display:
    {
        'overall': {
            'peak_freq_hz': dominant breathing frequency across all links
            'peak_bpm': peak frequency * 60 (breaths per minute)
            'in_adult_breathing_range': bool
            'total_active_subcarriers': count above ratio threshold
            'total_subcarriers': total
            'active_fraction': ratio
        },
        'per_link': {
            link: {
                'peak_freq_hz': dominant freq in breathing band
                'peak_power': spectral magnitude at peak
                'mean_band_ratio': avg breathing-band power ratio
                'n_active_subcarriers': sc count above threshold
                'n_subcarriers': total sc count
                'active_fraction': fraction
            }
        }
    }

    All math identical to per_link_doppler_power but returns richer
    per-link structure suitable for live status display.
    """
    per_link_samples = defaultdict(list)
    for r in window_records:
        if r.get("type") != "iq":
            continue
        link = _link_key(r.get("node_id"), r.get("peer_id"))
        if link is None:
            continue
        csi = _parse_iq_complex(r)
        if csi.size == 0:
            continue
        t = float(r.get("t") if r.get("t") is not None else r.get("_t_rel", 0.0))
        per_link_samples[link].append((t, np.abs(csi).astype(np.float64)))

    per_link = {}
    all_peak_freqs = []
    total_active = 0
    total_sc = 0

    for link, samples in per_link_samples.items():
        if len(samples) < min_packets:
            continue
        samples.sort(key=lambda s: s[0])
        ts = np.array([s[0] for s in samples])
        lens = [s[1].shape[0] for s in samples]
        common_len = int(np.median(lens))
        amps_list = [s[1][:common_len] for s in samples
                     if s[1].shape[0] >= common_len]
        if len(amps_list) < min_packets:
            continue
        amps = np.stack(amps_list, axis=0)
        n_pkts, n_sc = amps.shape
        duration = ts[-1] - ts[0]
        if duration < min_duration_s:
            continue
        fs = (n_pkts - 1) / duration

        # Hampel + polynomial detrend + Hanning (same pipeline as power func)
        med_t = np.median(amps, axis=0)
        mad_t = np.median(np.abs(amps - med_t[None, :]), axis=0)
        sigma_t = 1.4826 * mad_t
        deviation = np.abs(amps - med_t[None, :])
        outliers = (sigma_t[None, :] > 0) & (deviation > MAD_K * sigma_t[None, :])
        if outliers.any():
            amps = np.where(outliers,
                            np.broadcast_to(med_t[None, :], amps.shape),
                            amps)
        x = np.linspace(-1.0, 1.0, n_pkts).astype(np.float64)
        basis = np.vstack([x ** k for k in range(detrend_order + 1)]).T
        try:
            coeffs, *_ = np.linalg.lstsq(basis, amps, rcond=None)
            amps_dt = amps - basis @ coeffs
        except np.linalg.LinAlgError:
            amps_dt = amps - amps.mean(axis=0)[None, :]
        win = np.hanning(n_pkts).astype(np.float64).reshape(-1, 1)
        amps_w = amps_dt * win

        spectrum = np.abs(np.fft.rfft(amps_w, axis=0))  # (n_freqs, n_sc)
        freqs = np.fft.rfftfreq(n_pkts, 1.0 / fs)
        in_band = (freqs >= breathing_band[0]) & (freqs <= breathing_band[1])
        all_band = freqs > 0
        breathing_power = (spectrum[in_band] ** 2).sum(axis=0)
        total_power = (spectrum[all_band] ** 2).sum(axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(total_power > 0,
                              breathing_power / total_power, 0.0)
        n_active = int((ratio > ratio_threshold).sum())

        # Peak in breathing band: average spectrum across subcarriers,
        # find max in band. This is the dominant breathing frequency
        # across this link's subcarriers.
        avg_spec = spectrum.mean(axis=1)  # (n_freqs,)
        if in_band.any():
            band_freqs = freqs[in_band]
            band_spec = avg_spec[in_band]
            peak_idx = int(np.argmax(band_spec))
            peak_freq = float(band_freqs[peak_idx])
            peak_power = float(band_spec[peak_idx])
        else:
            peak_freq = 0.0
            peak_power = 0.0

        per_link[link] = {
            "peak_freq_hz": peak_freq,
            "peak_bpm": peak_freq * 60.0,
            "peak_power": peak_power,
            "mean_band_ratio": float(np.mean(ratio)),
            "n_active_subcarriers": n_active,
            "n_subcarriers": int(n_sc),
            "active_fraction": float(n_active) / float(n_sc) if n_sc else 0.0,
            "n_packets": int(n_pkts),
        }
        all_peak_freqs.append(peak_freq)
        total_active += n_active
        total_sc += n_sc

    overall = {}
    if all_peak_freqs:
        # Overall peak: use median of per-link peaks (robust)
        peak_overall = float(np.median(all_peak_freqs))
        overall = {
            "peak_freq_hz": peak_overall,
            "peak_bpm": peak_overall * 60.0,
            "in_adult_breathing_range": (0.20 <= peak_overall <= 0.40),
            "total_active_subcarriers": total_active,
            "total_subcarriers": total_sc,
            "active_fraction": (total_active / total_sc) if total_sc else 0.0,
        }
    return {"overall": overall, "per_link": per_link}


def per_link_doppler_robust(window_records,
                              breathing_band=BREATHING_BAND_HZ,
                              min_packets=8,
                              min_duration_s=BREATHING_MIN_DURATION_S,
                              detrend_order=4,
                              hampel_k=MAD_K,
                              n_welch_segments=4,
                              exclude_band_edge_bins=1,
                              highpass_cutoff_hz=0.10,
                              svd_drift_modes=0):
    """Robust Doppler features per link, baseline-free.

    Improvements over per_link_doppler_power:
      1. **Cross-subcarrier coherent summation**: sums |FFT|^2 across
         all subcarriers BEFORE peak finding. Body breathing at frequency
         f modulates many subcarriers, all peaking at the same f. The
         coherent sum aligns those peaks while uncorrelated subcarrier
         noise averages out. Sharper peak detection.

      2. **Welch's method** (4 segments, 50% overlap): splits the 90s
         window into 4 overlapping 36s segments, FFTs each, averages the
         power spectra. Reduces spectral variance by ~2x at the cost of
         frequency resolution (0.028 Hz per bin vs 0.011 Hz).

      3. **Peak SNR metric** (peak_power / median_power_in_band): the
         body breathing produces a SHARP narrow peak. Empty room produces
         a flat spectrum. Peak SNR > 5 typically indicates real breathing;
         < 2 is noise.

      4. **Wider breathing band (0.10 - 0.50 Hz)**: catches both slow
         (athletes, deep breathing) and fast (stressed) breathing rates.
         Combined with peak-SNR filtering, the wider band doesn't add
         noise because we're looking at the PEAK, not integrated power.

    Returns dict[link] -> {
        'peak_freq_hz': float,         # dominant frequency in band
        'peak_bpm': float,             # peak * 60 (breaths/min)
        'peak_power': float,           # spectral magnitude at peak
        'peak_snr': float,             # peak / median(band) - main signal
        'noise_floor': float,          # median power in band
        'spectrum': (n_freqs,) ndarray # for downstream visualization
        'freqs': (n_freqs,) ndarray
    }
    """
    per_link_samples = defaultdict(list)
    for r in window_records:
        if r.get("type") != "iq":
            continue
        link = _link_key(r.get("node_id"), r.get("peer_id"))
        if link is None:
            continue
        csi = _parse_iq_complex(r)
        if csi.size == 0:
            continue
        t = float(r.get("t") if r.get("t") is not None else r.get("_t_rel", 0.0))
        per_link_samples[link].append((t, np.abs(csi).astype(np.float64)))

    out = {}
    for link, samples in per_link_samples.items():
        if len(samples) < min_packets:
            continue
        samples.sort(key=lambda s: s[0])
        ts = np.array([s[0] for s in samples])
        lens = [s[1].shape[0] for s in samples]
        common_len = int(np.median(lens))
        amps_list = [s[1][:common_len] for s in samples
                     if s[1].shape[0] >= common_len]
        if len(amps_list) < min_packets:
            continue
        amps = np.stack(amps_list, axis=0)
        n_pkts, n_sc = amps.shape
        duration = ts[-1] - ts[0]
        if duration < min_duration_s:
            continue
        fs = (n_pkts - 1) / duration

        # CARM-style SVD denoising: drop top-K temporal modes (drift)
        # before any other processing. Operates on the (n_pkts, n_sc)
        # log-amplitude matrix per link.
        if svd_drift_modes > 0:
            amps = _svd_denoise_amps(amps, n_drift_modes=svd_drift_modes,
                                       log_amplitude=True)

        # Hampel + polynomial detrend (same as before)
        med_t = np.median(amps, axis=0)
        mad_t = np.median(np.abs(amps - med_t[None, :]), axis=0)
        sigma_t = 1.4826 * mad_t
        deviation = np.abs(amps - med_t[None, :])
        outliers = (sigma_t[None, :] > 0) & (deviation > hampel_k * sigma_t[None, :])
        if outliers.any():
            amps = np.where(outliers,
                            np.broadcast_to(med_t[None, :], amps.shape),
                            amps)
        x = np.linspace(-1.0, 1.0, n_pkts).astype(np.float64)
        basis = np.vstack([x ** k for k in range(detrend_order + 1)]).T
        try:
            coeffs, *_ = np.linalg.lstsq(basis, amps, rcond=None)
            amps_dt = amps - basis @ coeffs
        except np.linalg.LinAlgError:
            amps_dt = amps - amps.mean(axis=0)[None, :]

        # Path 2: 2nd-order Butterworth high-pass at 0.10 Hz to kill drift
        # residue that polynomial detrending leaves behind. Implemented as
        # a difference-equation 2nd-order IIR (no scipy dependency).
        if highpass_cutoff_hz > 0 and fs > 2 * highpass_cutoff_hz:
            # Bilinear transform of 2nd-order Butterworth high-pass
            wn = highpass_cutoff_hz / (fs / 2)  # normalized cutoff
            wc = np.tan(np.pi * wn / 2)
            k_h = 1.0 / (1.0 + np.sqrt(2) * wc + wc * wc)
            b0_h = k_h
            b1_h = -2.0 * k_h
            b2_h = k_h
            a1_h = 2.0 * (wc * wc - 1.0) * k_h
            a2_h = (1.0 - np.sqrt(2) * wc + wc * wc) * k_h
            # Apply forward and backward (zero-phase) for clean response
            for axis_pass in range(2):
                buf = np.zeros((2, amps_dt.shape[1]), dtype=np.float64)
                filt_buf = np.zeros_like(amps_dt)
                src = amps_dt if axis_pass == 0 else amps_dt[::-1]
                for i in range(src.shape[0]):
                    x_i = src[i]
                    y_i = b0_h * x_i + buf[0]
                    buf[0] = b1_h * x_i - a1_h * y_i + buf[1]
                    buf[1] = b2_h * x_i - a2_h * y_i
                    filt_buf[i] = y_i
                amps_dt = filt_buf if axis_pass == 0 else filt_buf[::-1]

        # Welch's method: split into K overlapping segments, FFT each,
        # average power spectra. K=n_welch_segments with 50% overlap.
        if n_welch_segments >= 2:
            seg_len = n_pkts // ((n_welch_segments + 1) // 2 + 1)
            # Use n_welch_segments segments of length seg_len with 50% overlap
            seg_len = max(seg_len, n_pkts // n_welch_segments)
            hop = seg_len // 2
            psds = []
            for i in range(n_welch_segments):
                start = i * hop
                end = start + seg_len
                if end > n_pkts:
                    break
                seg = amps_dt[start:end]
                # Hanning window per segment
                win = np.hanning(seg.shape[0]).astype(np.float64).reshape(-1, 1)
                seg_w = seg * win
                # FFT and accumulate power; sum across subcarriers (coherent)
                spec = np.abs(np.fft.rfft(seg_w, axis=0)) ** 2
                psds.append(spec.sum(axis=1))  # (n_freqs,) sum across sc
            if psds:
                psd = np.mean(np.stack(psds, axis=0), axis=0)
                freqs = np.fft.rfftfreq(seg_len, 1.0 / fs)
            else:
                # Fallback: single FFT
                win = np.hanning(n_pkts).astype(np.float64).reshape(-1, 1)
                spec = np.abs(np.fft.rfft(amps_dt * win, axis=0)) ** 2
                psd = spec.sum(axis=1)
                freqs = np.fft.rfftfreq(n_pkts, 1.0 / fs)
        else:
            # Single FFT, no Welch
            win = np.hanning(n_pkts).astype(np.float64).reshape(-1, 1)
            spec = np.abs(np.fft.rfft(amps_dt * win, axis=0)) ** 2
            psd = spec.sum(axis=1)
            freqs = np.fft.rfftfreq(n_pkts, 1.0 / fs)

        # Find peak in band
        in_band = (freqs >= breathing_band[0]) & (freqs <= breathing_band[1])
        if in_band.any():
            band_freqs = freqs[in_band]
            band_psd = psd[in_band]
            peak_idx = int(np.argmax(band_psd))
            peak_freq = float(band_freqs[peak_idx])
            peak_power = float(band_psd[peak_idx])
            noise_floor = float(np.median(band_psd))
            if noise_floor > 0:
                peak_snr = peak_power / noise_floor
            else:
                peak_snr = 0.0
        else:
            peak_freq = 0.0
            peak_power = 0.0
            noise_floor = 0.0
            peak_snr = 0.0

        out[link] = {
            "peak_freq_hz": peak_freq,
            "peak_bpm": peak_freq * 60.0,
            "peak_power": peak_power,
            "peak_snr": peak_snr,
            "noise_floor": noise_floor,
            "spectrum": psd,
            "freqs": freqs,
            "n_packets": int(n_pkts),
        }
    return out


class DopplerAccumulator:
    """Multi-window coherent integration of per-link Doppler spectra.

    Maintains a rolling deque of the last K per-link power spectra.
    When asked for a snapshot, AVERAGES the spectra coherently
    (assumes peak frequencies stay constant across windows for
    stationary bodies).

    SNR gain: a persistent breathing peak adds linearly across K
    windows; uncorrelated noise averages by 1/sqrt(K). Net peak SNR
    improvement: sqrt(K).

    Trade-off: response time = K * stride. For stationary subjects
    held in one quadrant for 5+ minutes, K=6 gives a 2.4x SNR gain at
    the cost of ~30 s of response latency.
    """

    def __init__(self, max_windows=6):
        self.max_windows = max_windows
        # link -> deque of {'spectrum': ndarray, 'freqs': ndarray, 't': float}
        self._history = {}
        # peak frequency persistence: link -> deque of peak freq Hz
        self._peak_history = {}

    def reset(self):
        """Drop all accumulated per-link spectra and peak history.

        After reset, the next K windows' integrated_features() will only
        contain post-reset evidence (where K = max_windows). This is the
        accumulator-level half of an operator soft-reset: the IQ buffer
        in live_iter holds the raw records; this object holds the
        derived multi-window-averaged spectra. Both must be dropped to
        prevent stale evidence from biasing the post-reset HMM.
        """
        self._history.clear()
        self._peak_history.clear()

    def push(self, robust_features, t_wall, min_peak_snr_for_history=1.5):
        """Add the current window's robust features to the accumulator.

        Only append peak frequency to peak_history when the peak is
        meaningful (peak_snr above threshold). Otherwise push None as
        a sentinel so the persistence count doesn't include phantom
        peaks from quiet links (where np.argmax over band noise picks
        a stable but spurious bin).
        """
        for link, info in robust_features.items():
            if link not in self._history:
                self._history[link] = deque(maxlen=self.max_windows)
                self._peak_history[link] = deque(maxlen=self.max_windows)
            self._history[link].append({
                "spectrum": info["spectrum"].copy(),
                "freqs": info["freqs"].copy(),
                "t": t_wall,
                "n_packets": info.get("n_packets", 0),
            })
            peak_snr = info.get("peak_snr", 0.0)
            if peak_snr >= min_peak_snr_for_history:
                self._peak_history[link].append(info["peak_freq_hz"])
            else:
                self._peak_history[link].append(None)

    def integrated_features(self, breathing_band=BREATHING_BAND_HZ,
                              freq_persistence_tolerance_hz=0.04,
                              min_persistence_count=2):
        """Compute integrated per-link features from the accumulated history.

        Returns dict[link] -> {
            'peak_freq_hz', 'peak_bpm', 'peak_power', 'peak_snr',
            'noise_floor', 'spectrum', 'freqs',
            'n_windows_integrated': int,
            'peak_persistence_count': int  # how many recent windows
                                            # had the same peak (within tol)
        }
        """
        out = {}
        for link, hist in self._history.items():
            if not hist:
                continue
            # Average spectra (assume same freq bins across windows;
            # they should be since fs and n_pkts are stable)
            base_freqs = hist[-1]["freqs"]
            specs = [h["spectrum"] for h in hist
                     if h["spectrum"].shape == base_freqs.shape]
            if not specs:
                continue
            avg_spec = np.mean(np.stack(specs, axis=0), axis=0)

            in_band = (base_freqs >= breathing_band[0]) & (base_freqs <= breathing_band[1])
            if not in_band.any():
                continue
            band_freqs = base_freqs[in_band]
            band_spec = avg_spec[in_band]
            peak_idx = int(np.argmax(band_spec))
            peak_freq = float(band_freqs[peak_idx])
            peak_power = float(band_spec[peak_idx])
            noise_floor = float(np.median(band_spec))
            peak_snr = peak_power / noise_floor if noise_floor > 0 else 0.0

            # Peak persistence count: how many recent windows had a
            # peak within the tolerance of this window's peak. Skip
            # None entries (windows where peak SNR was too low to
            # count as a real detection).
            persistence_count = 0
            recent_peaks = list(self._peak_history.get(link, []))
            for pf in recent_peaks:
                if pf is None:
                    continue
                if abs(pf - peak_freq) <= freq_persistence_tolerance_hz:
                    persistence_count += 1

            # Average n_packets across history entries (latest matters
            # most for filtering decisions, but the average reflects
            # sustained packet quality)
            n_pkts_avg = float(np.mean([h.get("n_packets", 0) for h in hist]))
            out[link] = {
                "peak_freq_hz": peak_freq,
                "peak_bpm": peak_freq * 60.0,
                "peak_power": peak_power,
                "peak_snr": peak_snr,
                "noise_floor": noise_floor,
                "spectrum": avg_spec,
                "freqs": base_freqs,
                "n_windows_integrated": len(specs),
                "peak_persistence_count": persistence_count,
                "n_packets_avg": n_pkts_avg,
            }
        return out


def detect_bodies_from_doppler(robust_features,
                                 min_peak_snr=4.0,
                                 freq_cluster_tolerance_hz=0.04,
                                 min_persistence_count=4,
                                 min_persistence_snr=1.5,
                                 merge_shared_node=True,
                                 single_body=False,
                                 min_packets_per_link=60):
    """Cluster per-link breathing peaks into 'bodies' by frequency.

    Multiple bodies in a room produce multiple distinct breathing
    frequencies. Each body's frequency is seen on the links its
    multipath modulates. Cluster links by their peak frequency:
    same cluster = same body.

    Returns list of body dicts:
        [{
            'frequency_hz': float,           # cluster center
            'bpm': float,
            'links': [(a, b), ...],         # links seeing this body
            'mean_peak_snr': float,
            'dominant_node': int|None,       # node ID most touched
        }]

    Sorted by mean_peak_snr descending (strongest body first).
    """
    if not robust_features:
        return []
    # Filter starved links: their sparse FFT can produce phantom-high
    # peak SNR by chance, polluting body detection. Same logic as
    # dominant_node_from_doppler / format_doppler_explanation.
    if min_packets_per_link > 0:
        robust_features = {
            l: info for l, info in robust_features.items()
            if (info.get("n_packets_avg", info.get("n_packets", 0))
                >= min_packets_per_link)
        }
        if not robust_features:
            return []
    # Each link has a peak frequency and SNR. A link counts as
    # "showing breathing" if EITHER:
    #   - peak_snr >= min_peak_snr (single-window strong signal), OR
    #   - peak_persistence_count >= min_persistence_count AND
    #     peak_snr >= min_persistence_snr (consistent weak signal across
    #     multiple windows = real signal that just happens to be weak).
    candidates = []
    for link, info in robust_features.items():
        snr = info["peak_snr"]
        persist = info.get("peak_persistence_count", 0)
        if snr >= min_peak_snr:
            candidates.append((link, info["peak_freq_hz"], snr))
        elif persist >= min_persistence_count and snr >= min_persistence_snr:
            # Multi-window persistence: weak per-window but consistent.
            # Score as snr * sqrt(persist) for ranking.
            effective_snr = snr * (persist ** 0.5)
            candidates.append((link, info["peak_freq_hz"], effective_snr))
    if not candidates:
        return []

    # Greedy clustering by frequency proximity
    candidates.sort(key=lambda x: x[1])  # sort by frequency
    clusters = []
    for link, freq, snr in candidates:
        placed = False
        for cluster in clusters:
            if abs(cluster["mean_freq"] - freq) <= freq_cluster_tolerance_hz:
                cluster["links"].append(link)
                cluster["snrs"].append(snr)
                cluster["freqs"].append(freq)
                cluster["mean_freq"] = float(np.mean(cluster["freqs"]))
                placed = True
                break
        if not placed:
            clusters.append({
                "mean_freq": freq,
                "links": [link],
                "snrs": [snr],
                "freqs": [freq],
            })

    # For each cluster, compute dominant node by binary tally with
    # SNR-weighted tie-break. Pair with temporal smoothing in the
    # caller for stable display.
    bodies = []
    for c in clusters:
        counts = {1: 0, 2: 0, 3: 0, 4: 0}
        snr_sum = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        for (a, b), snr in zip(c["links"], c["snrs"]):
            counts[a] += 1
            counts[b] += 1
            snr_sum[a] += snr
            snr_sum[b] += snr
        dominant_node = None
        # Sort by (count desc, snr_sum desc): ties broken by total SNR
        # contribution from that node's links
        nodes_ranked = sorted(counts.keys(),
                               key=lambda n: (-counts[n], -snr_sum[n]))
        best = nodes_ranked[0]
        if counts[best] >= 2:
            dominant_node = best
        bodies.append({
            "frequency_hz": c["mean_freq"],
            "bpm": c["mean_freq"] * 60.0,
            "links": c["links"],
            "mean_peak_snr": float(np.mean(c["snrs"])),
            "dominant_node": dominant_node,
        })
    bodies.sort(key=lambda b: -b["mean_peak_snr"])

    # Post-process: merge clusters that share at least one node. A
    # single physical body always touches multiple links via its
    # multipath, so two clusters sharing a node are almost certainly
    # the same body whose peak frequency jittered across links.
    if merge_shared_node and len(bodies) > 1:
        merged = []
        used = [False] * len(bodies)
        for i, b in enumerate(bodies):
            if used[i]:
                continue
            cur_links = set(b["links"])
            cur_nodes = {n for (a, c) in cur_links for n in (a, c)}
            cur = dict(b)
            for j in range(i + 1, len(bodies)):
                if used[j]:
                    continue
                other = bodies[j]
                other_nodes = {n for (a, c) in other["links"] for n in (a, c)}
                if cur_nodes & other_nodes:
                    # Merge: union links, weighted-average frequency by SNR
                    w1 = cur["mean_peak_snr"]
                    w2 = other["mean_peak_snr"]
                    if w1 + w2 > 0:
                        cur["frequency_hz"] = (
                            (cur["frequency_hz"] * w1
                             + other["frequency_hz"] * w2) / (w1 + w2)
                        )
                        cur["bpm"] = cur["frequency_hz"] * 60.0
                    cur_links = cur_links | set(other["links"])
                    cur["links"] = list(cur_links)
                    cur_nodes |= other_nodes
                    cur["mean_peak_snr"] = max(w1, w2)
                    # Recompute dominant node from union
                    nodes_list = []
                    for (a, c) in cur_links:
                        nodes_list.extend([a, c])
                    counts = {n: nodes_list.count(n) for n in set(nodes_list)}
                    best = max(counts, key=counts.get)
                    cur["dominant_node"] = (best if counts[best] >= 2
                                              else None)
                    used[j] = True
            merged.append(cur)
            used[i] = True
        bodies = merged

    if single_body and len(bodies) > 1:
        bodies = bodies[:1]  # already sorted by SNR descending
    return bodies


def format_robust_doppler(robust_features, bodies):
    """Format the robust Doppler analysis for live display.

    Shows per-body breakdown (separated by frequency clustering) so
    multi-body rooms produce multi-body output:

        [doppler-robust] body 1: 17 BPM (peak SNR 8.3) on L24, L34, L14 -> node 4 (Q4)
        [doppler-robust] body 2: 14 BPM (peak SNR 5.1) on L12, L23     -> node 2 (Q1)
        [doppler-robust] empty/quiet links: L13
    """
    if not bodies:
        return "[doppler-robust] no breathing peaks detected (room may be empty)"
    lines = []
    node_to_q = {1: "Q3 (SW)", 2: "Q1 (NW)", 3: "Q2 (NE)", 4: "Q4 (SE)"}
    for i, body in enumerate(bodies, 1):
        link_str = ", ".join(f"L{a}{b}" for (a, b) in body["links"])
        node_str = (f"-> node {body['dominant_node']} = "
                    f"{node_to_q.get(body['dominant_node'], '?')}"
                    if body['dominant_node'] is not None else "(no clear node)")
        lines.append(
            f"[doppler-robust] body {i}: {body['bpm']:.0f} BPM "
            f"(peak SNR {body['mean_peak_snr']:.1f}) on {link_str}  {node_str}"
        )
    # List quiet links (high peak SNR not detected)
    used_links = {l for body in bodies for l in body["links"]}
    quiet = [(a, b) for (a, b) in ALL_LINKS if (a, b) not in used_links]
    if quiet:
        quiet_str = ", ".join(f"L{a}{b}" for (a, b) in quiet)
        lines.append(f"[doppler-robust] quiet links (no breathing): {quiet_str}")
    return "\n".join(lines)


def dominant_node_from_doppler(explain, top_k=3,
                                 adaptive_filter_pct=0.50,
                                 min_packets_floor=70,
                                 min_surviving_links=4):
    """Return the node ID (1..4) most represented in the top-K Doppler-
    active links, or None if conditions for a confident call aren't met.

    Binary top-K tally sorted by active_fraction, with two reliability
    gates that handle non-stationary SoftAP queue:

    GATE 1 — Adaptive starved-link filter:
        Compute median packet count across all 6 links. Drop any link
        below max(`min_packets_floor`, `adaptive_filter_pct` * median).
        Adapts to whatever the queue is doing — if all links are at
        150, filter cuts at 45; if degraded to 80, filter cuts at 24.

    GATE 2 — Minimum surviving links:
        After filter, require at least `min_surviving_links` to remain.
        With 4 nodes and 6 links, at least 4 links are needed to
        triangulate a position confidently. Below that threshold,
        return None (caller should display ANALYZING — not enough
        data to call confidently).
    """
    per_link = explain.get("per_link", {}) if explain else {}
    if not per_link:
        return None
    # GATE 1: adaptive starved-link filter
    pkt_counts = [info.get("n_packets", 0) for info in per_link.values()]
    if pkt_counts:
        median_pkts = sorted(pkt_counts)[len(pkt_counts) // 2]
        threshold = max(min_packets_floor,
                        adaptive_filter_pct * median_pkts)
        per_link = {l: info for l, info in per_link.items()
                    if info.get("n_packets", 0) >= threshold}
    # GATE 2: insufficient surviving links -> refuse to call
    if len(per_link) < min_surviving_links:
        return None
    links_sorted = sorted(per_link.items(),
                           key=lambda kv: -kv[1].get("active_fraction", 0))
    top = links_sorted[:top_k]
    if not top:
        return None
    # Tally nodes; break ties by total active_fraction across incident
    # links so a non-deterministic dict iteration order does not bias
    # toward low-numbered nodes.
    counts = {1: 0, 2: 0, 3: 0, 4: 0}
    activity_sum = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
    for (a, b), info in top:
        counts[a] += 1
        counts[b] += 1
        af = info.get("active_fraction", 0.0)
        activity_sum[a] += af
        activity_sum[b] += af
    # Sort by (count desc, activity_sum desc) -- ties broken by total
    # signal contribution from that node's links
    nodes_ranked = sorted(counts.keys(),
                           key=lambda n: (-counts[n], -activity_sum[n]))
    best = nodes_ranked[0]
    if counts[best] < 2:
        return None  # no clear dominance
    return best


_CLASS_TO_NODE = {"Q1": 2, "Q2": 3, "Q3": 1, "Q4": 4}


class DopplerPredictionSmoother:
    """Temporal majority-vote smoother for the [doppler] dominant-node
    prediction.

    Per-window calibrationless predictions inherently flicker because
    Doppler-band activity on different links varies window to window
    even for a stationary body. A running majority vote over the last
    N predictions converts a noisy ~60% per-window predictor into a
    stable ~95% time-averaged predictor.

    Usage:
        sm = DopplerPredictionSmoother(window_size=30, min_confidence=0.50)
        for emission in detector_loop:
            raw = ... # dominant node from current window (1..4 or None)
            smoothed = sm.update(raw)  # returns (node|None, confidence)
            display(smoothed)
    """

    def __init__(self, window_size=30, min_confidence=0.50):
        self.window_size = window_size
        self.min_confidence = min_confidence
        self._history = deque(maxlen=window_size)

    def update(self, node):
        """Push current prediction (None or 1..4), return (node, confidence).

        Always append (even None), so the deque tracks total emissions.
        Confidence = best_count / total_calls — a stretch of None
        inputs decays confidence below threshold and the smoother
        emits None (ANALYZING) instead of stale stuck predictions.
        """
        self._history.append(node)
        if not self._history:
            return None, 0.0
        counts = {n: 0 for n in (1, 2, 3, 4)}
        n_valid = 0
        for v in self._history:
            if v is None:
                continue
            counts[v] = counts.get(v, 0) + 1
            n_valid += 1
        if n_valid == 0:
            return None, 0.0
        # Confidence denominator is total deque length (including
        # None placeholders) -- this decays stale predictions when
        # bodies leave the room.
        best = max(counts, key=counts.get)
        confidence = counts[best] / len(self._history)
        if confidence < self.min_confidence:
            return None, confidence
        return best, confidence

    def reset(self):
        self._history.clear()


def format_doppler_explanation(explain,
                                 adaptive_filter_pct=0.50,
                                 min_packets_floor=70,
                                 min_surviving_links=4,
                                 use_continuous=False):
    """Format the doppler self-explanation for live display.

    Returns a multi-line string ready to print. Uses the same adaptive
    starved-link filter and minimum-surviving-links gate as
    dominant_node_from_doppler so the displayed top-3 and dominant-node
    line are consistent.
    """
    overall = explain.get("overall", {})
    per_link_full = explain.get("per_link", {})
    if not overall or not per_link_full:
        return "[doppler] insufficient data for breathing analysis"

    lines = []
    bpm = overall.get("peak_bpm", 0.0)
    in_range = overall.get("in_adult_breathing_range", False)
    range_note = "adult resting range" if in_range else "outside resting band"
    lines.append(
        f"[doppler] peak: {overall['peak_freq_hz']:.2f} Hz "
        f"({bpm:.0f} breaths/min, {range_note})"
    )
    lines.append(
        f"[doppler] active subcarriers: "
        f"{overall['total_active_subcarriers']} / "
        f"{overall['total_subcarriers']} "
        f"({overall['active_fraction']*100:.1f}%)"
    )
    # GATE 1: adaptive starved-link filter
    pkt_counts = [info.get("n_packets", 0)
                  for info in per_link_full.values()]
    if pkt_counts:
        median_pkts = sorted(pkt_counts)[len(pkt_counts) // 2]
        threshold = max(min_packets_floor,
                        adaptive_filter_pct * median_pkts)
        per_link = {l: info for l, info in per_link_full.items()
                    if info.get("n_packets", 0) >= threshold}
        n_excluded = len(per_link_full) - len(per_link)
        if n_excluded > 0:
            excluded = [f"L{a}{b}" for (a, b), info in per_link_full.items()
                        if info.get("n_packets", 0) < threshold]
            lines.append(
                f"[doppler] excluded {n_excluded} starved link(s): "
                f"{', '.join(excluded)} (median={median_pkts} pkts, "
                f"cutoff={threshold:.0f})"
            )
    else:
        per_link = per_link_full
    # GATE 2: minimum surviving links
    if len(per_link) < min_surviving_links:
        lines.append(
            f"[doppler] only {len(per_link)} healthy link(s) -- need "
            f"{min_surviving_links}+ to triangulate. ANALYZING."
        )
        return "\n".join(lines)
    if not per_link:
        lines.append("[doppler] all links starved - no dominant node")
        return "\n".join(lines)
    # Top 3 links by active_fraction (matches dominant-node-from-doppler)
    links_sorted = sorted(per_link.items(),
                           key=lambda kv: -kv[1]["active_fraction"])
    top_links = links_sorted[:3]
    if top_links:
        link_str = "  ".join(
            f"L{a}{b}: {info['active_fraction']*100:.0f}% "
            f"@ {info['peak_freq_hz']:.2f} Hz"
            for (a, b), info in top_links
        )
        lines.append(f"[doppler] strongest links: {link_str}")
        node_to_q = {1: "Q3 (SW)", 2: "Q1 (NW)",
                     3: "Q2 (NE)", 4: "Q4 (SE)"}
        if use_continuous:
            # Soft endpoint attribution over ALL surviving links (not
            # just top-3). Matches the live HMM observation source so
            # the displayed dominant-node line is consistent with the
            # PREDICTION headline.
            dom_new, energy, conf = continuous_node_estimator(
                per_link, reliability_exp=0.5, min_confidence=0.30,
            )
            if dom_new is not None:
                lines.append(
                    f"[doppler] -> dominant node: {dom_new} = "
                    f"{node_to_q.get(dom_new, '?')}  "
                    f"(continuous, conf={conf*100:.0f}%)"
                )
        else:
            # Legacy binary top-K tally with active_fraction tiebreak.
            node_counts = {1: 0, 2: 0, 3: 0, 4: 0}
            node_activity = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
            for (a, b), info in top_links:
                node_counts[a] += 1
                node_counts[b] += 1
                af = info.get("active_fraction", 0.0)
                node_activity[a] += af
                node_activity[b] += af
            nodes_ranked = sorted(node_counts.keys(),
                                   key=lambda n: (-node_counts[n],
                                                  -node_activity[n]))
            node_dominant = nodes_ranked[0]
            if node_counts[node_dominant] >= 2:
                lines.append(
                    f"[doppler] -> dominant node: {node_dominant} = "
                    f"{node_to_q.get(node_dominant, '?')}"
                )
    return "\n".join(lines)


# --------------------------------------------------------------------------
# Output formatting
# --------------------------------------------------------------------------
def format_prediction(predicted_class, deltas, ts_wall=None, n_records=0,
                      rule_index=None):
    if ts_wall is None:
        ts_wall = time.time()
    hms = time.strftime("%H:%M:%S", time.localtime(ts_wall))
    rule_str = f" rule={rule_index:>2}" if rule_index is not None else ""
    # Calibrationless / DOPPLER_ONLY mode: deltas is empty, every link
    # would print as NaN. Suppress the per-link line entirely.
    if not deltas:
        return (f"[{hms}] {predicted_class}{rule_str}  "
                f"n_records={n_records}")
    # Show per-link mean delta for a quick sanity read.
    per_link_mu = {l: float(deltas[l].mean()) if l in deltas else float("nan")
                   for l in ALL_LINKS}
    mu_str = "  ".join(
        f"L{a}{b}={per_link_mu[(a, b)]:+5.2f}" for (a, b) in ALL_LINKS
    )
    return (f"[{hms}] {predicted_class}{rule_str}  n_records={n_records}  "
            f"per-link mean-delta: {mu_str}")


# --------------------------------------------------------------------------
# Background Doppler compute worker
# --------------------------------------------------------------------------
class _DopplerComputeWorker(threading.Thread):
    """Run the heavy Doppler/SVD/body-detect path off the main UDP loop.

    Numpy FFT (np.fft.rfft) and SVD (np.linalg.svd) release the GIL for the
    duration of their underlying BLAS/LAPACK calls. So this worker truly runs
    in parallel with main-thread Python (UDP receive, packet parsing, buffer
    appends) on a multi-core machine. Without this thread the receive loop
    would block for ~150-300ms on every emission while Welch + SVD finishes,
    overflowing the OS UDP socket buffer at ~250 pkts/s and silently
    dropping packets.

    The worker owns the DopplerAccumulator (multi-window coherent
    integration) so its history is consistent across submissions.

    Backpressure: the in-queue is bounded to 2 entries. If main thread
    submits faster than worker drains, the OLDEST pending request is
    dropped -- we always want the freshest Doppler result, not a stale
    one. The result queue is unbounded; main thread polls it every
    emission, so it cannot back up significantly.
    """

    def __init__(self, accumulator_max_windows=6,
                 breathing_ratio_threshold=BREATHING_BAND_RATIO_THRESHOLD):
        super().__init__(daemon=True, name="doppler-compute")
        self._in_q = queue.Queue(maxsize=2)
        self._out_q = queue.Queue()
        self._stop_event = threading.Event()
        self._accum = DopplerAccumulator(max_windows=accumulator_max_windows)
        self._breathing_ratio_threshold = breathing_ratio_threshold

    def submit(self, request):
        try:
            self._in_q.put_nowait(request)
        except queue.Full:
            try:
                self._in_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._in_q.put_nowait(request)
            except queue.Full:
                pass

    def drain_results(self):
        results = []
        while True:
            try:
                results.append(self._out_q.get_nowait())
            except queue.Empty:
                break
        return results

    def stop(self):
        self._stop_event.set()
        try:
            self._in_q.put_nowait(None)
        except queue.Full:
            pass

    def reset(self):
        """Operator soft-reset: drop the multi-window accumulator and
        any pending in-flight requests/results.

        Called when the operator presses R between demo positions. Any
        request currently mid-compute will still finish and post a
        result; the caller must also re-clear `latest_dop_result` and
        ignore the next single drained result, otherwise stale features
        from before the reset can sneak through one window.
        """
        # Drain pending input requests so we don't process stale IQ.
        while True:
            try:
                self._in_q.get_nowait()
            except queue.Empty:
                break
        # Drain any results sitting in the output queue.
        while True:
            try:
                self._out_q.get_nowait()
            except queue.Empty:
                break
        # Drop the K-window coherent-integration history.
        self._accum.reset()

    def run(self):
        while not self._stop_event.is_set():
            try:
                req = self._in_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if req is None:
                break
            try:
                result = self._compute(req)
            except Exception as exc:
                result = {
                    "request_id": req.get("request_id"),
                    "t": req.get("t"),
                    "error": str(exc),
                }
            try:
                self._out_q.put_nowait(result)
            except queue.Full:
                pass

    def _compute(self, req):
        """Heavy compute path. NUMPY OPS RELEASE GIL — runs in parallel."""
        out = {
            "request_id": req["request_id"],
            "t": req["t"],
            "doppler_ready": False,
            "doppler": None,
            "robust_features": None,
            "bodies": None,
            "doppler_explain": None,
            "breathing_score": 0.0,
        }
        records = req.get("records") or []
        opts = req["opts"]
        if not records:
            return out
        dop_duration = (records[-1].get("_t_rel", 0.0)
                        - records[0].get("_t_rel", 0.0))
        if dop_duration < BREATHING_MIN_DURATION_S:
            return out
        out["doppler_ready"] = True

        # Compute primary Doppler (used for breathing_score and explain).
        doppler = per_link_doppler_power(
            records, min_duration_s=BREATHING_MIN_DURATION_S,
        )
        out["doppler"] = doppler
        if opts.get("breathing_presence", 0.0) > 0:
            out["breathing_score"] = breathing_presence_score(
                doppler,
                ratio_threshold=opts.get(
                    "breathing_ratio_threshold",
                    self._breathing_ratio_threshold,
                ),
            )
        if opts.get("explain_doppler"):
            out["doppler_explain"] = doppler_self_explain(
                records,
                ratio_threshold=opts.get(
                    "breathing_ratio_threshold",
                    self._breathing_ratio_threshold,
                ),
            )
        # Robust path: Welch + Butterworth + SVD denoise + accumulator
        if opts.get("robust_doppler") or opts.get("localize"):
            win_robust = per_link_doppler_robust(
                records,
                min_duration_s=BREATHING_MIN_DURATION_S,
                svd_drift_modes=opts.get("svd_drift_modes", 0),
            )
            self._accum.push(win_robust, req["t"])
            robust_features = self._accum.integrated_features()
            out["robust_features"] = robust_features
            if opts.get("robust_doppler"):
                out["bodies"] = detect_bodies_from_doppler(
                    robust_features,
                    min_peak_snr=opts.get("body_snr_threshold", 4.0),
                    single_body=opts.get("single_body", False),
                )
        return out


# --------------------------------------------------------------------------
# Live UDP driver
# --------------------------------------------------------------------------
def live_iter(baseline,
              bind_host="0.0.0.0",
              bind_port=4211,
              window_s=WINDOW_SECONDS,
              stride_s=ADVANCE_SECONDS,
              breathing_window_s=BREATHING_WINDOW_SECONDS,
              stop_event=None,
              stats_interval_s=2.0,
              quiet=False,
              empty_threshold=0.0,
              baseline_alpha=0.0,
              debug_log_fp=None,
              breathing_presence=0.0,
              breathing_ratio_threshold=BREATHING_BAND_RATIO_THRESHOLD,
              explain_doppler=False,
              robust_doppler=False,
              localize=False,
              accumulator_max_windows=6,
              position_smooth_alpha=0.0,
              body_snr_threshold=2.5,
              svd_drift_modes=0,
              single_body=False,
              continuous_estimator=False,
              hmm_p_stay=0.95,
              hmm_min_confidence=0.50,
              hmm_adjacency_weight=4.0,
              position_max_step_m=1.5,
              starvation_warn_threshold=30,
              starvation_warning_interval_s=20.0,
              starvation_min_packets=70,
              starvation_pct_of_median=0.5,
              per_link_normalize=False,
              per_link_normalize_alpha=0.02,
              use_blockage_signal=False,
              blockage_weight=0.30,
              min_link_packets=0,
              pygame_display=None):
    if stop_event is None:
        stop_event = threading.Event()
    from udp_receiver import UDPReceiver
    from frame_decoder import parse_packet

    rx = UDPReceiver(bind_host=bind_host, bind_port=bind_port)
    rx.open()
    if not quiet:
        print(f"[live] bound UDP {bind_host}:{bind_port}, awaiting packets...",
              file=sys.stderr, flush=True)

    buf = deque()             # short window (window_s) for variance + classify
    doppler_buf = deque()     # long window (breathing_window_s) for FFT
    first_t = None
    next_emit_t = None
    pkt_total = 0
    type_counts = {}
    iq_per_node = {1: 0, 2: 0, 3: 0, 4: 0}

    # Track per-node uptime to detect resets in the live stream and
    # exclude IQ records within +/- RESET_GUARD_SECONDS of any reset.
    last_uptime = {}
    reset_intervals = []  # list of (t_lo, t_hi) tuples in monotonic time
    # Most-recent reboot info for the pygame "LAST RESET" panel.
    # Populated from the telemetry frame whose uptime regressed.
    last_reset_info = None  # {"node_id", "reason_code", "reason_label",
                            #  "crash_count", "t_wall", "t_mono"} or None
    # Per-node crash counters (latest seen via telemetry, NOT just on
    # the reset event). The display reads this as a forensic record.
    per_node_crash_count = {1: 0, 2: 0, 3: 0, 4: 0}
    # Rolling history of recent reboots for the LAST RESET panel's
    # "Recent reboots" list. Kept short for live operator diagnosis;
    # the JSONL debug log retains the full audit trail.
    recent_resets = deque(maxlen=10)
    # Per-link signal-score baselines for --per-link-normalize.
    # EMA-updated each window; passed into continuous_node_estimator
    # so each link contributes its DEVIATION from its own baseline,
    # correcting for SoftAP TXQ bias that puts one diagonal at ~5x
    # the activity of the other. Two separate baselines: one for the
    # explain pipeline (active_fraction-driven), one for the robust
    # pipeline (peak_snr-driven), since the scales differ.
    per_link_baseline_explain: Dict[Tuple[int, int], float] = {}
    per_link_baseline_robust: Dict[Tuple[int, int], float] = {}

    t_wall_start = time.monotonic()
    t_next_stats = t_wall_start + stats_interval_s

    # The multi-window Doppler accumulator lives inside
    # _DopplerComputeWorker so it is not accessed concurrently from
    # the receive loop.
    # Activate position smoother if EITHER alpha smoothing OR kinematic
    # clamp is requested. With alpha=0 and max_step_m>0 we get pure
    # velocity clipping (no temporal averaging). With alpha>0 and
    # max_step_m=0 we get pure EMA (no clipping). Either suffices.
    _pos_alpha = position_smooth_alpha if position_smooth_alpha > 0 else 1.0
    _pos_max_step = (position_max_step_m
                     if position_max_step_m and position_max_step_m > 0
                     else None)
    pos_smoother = (localizer_2d.PositionSmoother(
                        alpha=_pos_alpha,
                        max_step_m=_pos_max_step,
                    )
                    if (localize and (position_smooth_alpha > 0
                                       or _pos_max_step is not None))
                    else None)

    # Background compute worker. Heavy Doppler / SVD / body-detect runs
    # in this thread so the main UDP receive loop never blocks.
    want_doppler_global = ((breathing_presence > 0)
                           or explain_doppler or robust_doppler or localize
                           or continuous_estimator)
    if continuous_estimator:
        # Compute both the explain pipeline (active_fraction) and the
        # robust pipeline (peak_snr) so the continuous estimator has
        # both inputs to combine.
        explain_doppler = True
        robust_doppler = True
    worker = None
    if want_doppler_global:
        worker = _DopplerComputeWorker(
            accumulator_max_windows=accumulator_max_windows,
            breathing_ratio_threshold=breathing_ratio_threshold,
        )
        worker.start()
    # Latest computed-by-worker fields. Reused across emissions until
    # a fresh result arrives. None until first compute completes.
    latest_dop_result = None
    next_request_id = 0

    # Temporal majority-vote smoother for calibrationless dominant-node
    # predictions. window=9 / min_confidence=0.50 balances emit rate
    # against per-window noise: short enough to track demo movement,
    # long enough to reject single-window flips.
    #
    # Per-link starvation streak tracker. Whenever a link is excluded
    # by the adaptive starvation filter (n_pkts < max(70, 0.5*median))
    # for many consecutive windows, the SoftAP queue is biased away
    # from that link. Empirically this manifests as steady predictions
    # that drift when the queue finally rotates: the rotation flips
    # the link's fingerprint in a way that fools the localizer/HMM.
    # Recommended action: restart coord to reset the queue.
    link_starve_streak = {f"{a}-{b}": 0 for (a, b) in ALL_LINKS}
    last_starve_warning_t = 0.0

    # Operator reset signal: when set by the stdin watcher thread, the
    # main loop resets the HMM posterior to uniform AND clears the
    # position smoother. Use this between demo positions ('r' + Enter
    # at the keyboard) so the smoother doesn't anchor on the last
    # quadrant when the body has just been moved.
    reset_request = threading.Event()

    def _stdin_reset_watcher():
        """Watch stdin for an 'r' keypress; signal a reset.

        Reads line by line so it works on Windows + Unix without
        platform-specific raw-mode tty handling. User types 'r' (or
        any line starting with r) + Enter to fire a reset.
        """
        try:
            for line in sys.stdin:
                if line.strip().lower().startswith("r"):
                    reset_request.set()
        except Exception:
            pass

    # Daemon so the watcher dies with the process; doesn't block exit.
    threading.Thread(target=_stdin_reset_watcher, daemon=True).start()
    if not quiet:
        print("[live] type 'r' + Enter at any time to reset the HMM "
              "posterior + position smoother (use between demo moves)",
              file=sys.stderr, flush=True)

    if continuous_estimator:
        # Forward-filter HMM smoother. Operates on continuous per-window
        # node energy vectors, integrating low-confidence observations
        # gracefully.
        dop_smoother = HmmNodeSmoother(
            p_stay=hmm_p_stay,
            min_confidence=hmm_min_confidence,
            adjacency_weight=hmm_adjacency_weight,
        )
        if not quiet:
            print(f"[live] continuous-estimator MODE: HMM smoother "
                  f"(p_stay={hmm_p_stay}, min_confidence={hmm_min_confidence}, "
                  f"adjacency_weight={hmm_adjacency_weight})",
                  file=sys.stderr, flush=True)
    else:
        dop_smoother = DopplerPredictionSmoother(window_size=9,
                                                    min_confidence=0.50)

    try:
        for payload in rx.read_packets():
            if stop_event.is_set():
                break
            rec = parse_packet(payload)
            pkt_total += 1
            tp = rec.get("type", "unknown")
            type_counts[tp] = type_counts.get(tp, 0) + 1

            if tp == "telemetry":
                # Detect reboots via uptime regression.
                nid = rec.get("node_id")
                upt = rec.get("uptime_s")
                _crash = rec.get("crash_count")
                if isinstance(nid, int) and isinstance(_crash, int):
                    if 1 <= nid <= 4:
                        per_node_crash_count[nid] = _crash
                if isinstance(nid, int) and isinstance(upt, int):
                    if nid in last_uptime and upt < last_uptime[nid]:
                        now = time.monotonic()
                        reset_intervals.append((now - RESET_GUARD_SECONDS,
                                                  now + RESET_GUARD_SECONDS))
                        # Capture the most-recent reset for the live
                        # pygame "LAST RESET" panel. The reason_code
                        # comes from esp_reset_reason() in the
                        # telemetry frame whose uptime regressed.
                        rcode = rec.get("reset_reason")
                        last_reset_info = {
                            "node_id": nid,
                            "reason_code": rcode,
                            "reason_label": reset_reason_label(rcode),
                            "crash_count": rec.get("crash_count"),
                            "t_wall": time.time(),
                            "t_mono": now,
                        }
                        recent_resets.append(dict(last_reset_info))
                        if debug_log_fp is not None:
                            debug_log_fp.write(json.dumps({
                                "type": "reset",
                                "t_wall": time.time(),
                                "node_id": nid,
                                "reset_reason": rcode,
                                "reset_reason_label": reset_reason_label(rcode),
                                "crash_count": rec.get("crash_count"),
                            }) + "\n")
                            debug_log_fp.flush()
                    last_uptime[nid] = upt

            if tp == "iq":
                nid = rec.get("node_id")
                if isinstance(nid, int) and nid in iq_per_node:
                    iq_per_node[nid] += 1

            now = time.monotonic()
            if now >= t_next_stats:
                if not quiet:
                    elapsed = now - t_wall_start
                    # Show Doppler warm-up progress if breathing gate
                    # is enabled and not yet warmed up.
                    dop_status = ""
                    if breathing_presence > 0 and len(doppler_buf) > 0:
                        dop_dur = (float(doppler_buf[-1].get("_t_rel", 0.0))
                                   - float(doppler_buf[0].get("_t_rel", 0.0)))
                        if dop_dur < BREATHING_MIN_DURATION_S:
                            dop_status = (f" WARMUP {dop_dur:4.1f}/"
                                          f"{BREATHING_MIN_DURATION_S:.0f}s")
                        else:
                            dop_status = " READY"
                    print(f"\r[live] {elapsed:5.1f}s  pkts={pkt_total:6d} "
                          f"iq={type_counts.get('iq',0):5d} "
                          f"[n1={iq_per_node[1]:4d} n2={iq_per_node[2]:4d} "
                          f"n3={iq_per_node[3]:4d} n4={iq_per_node[4]:4d}] "
                          f"buf={len(buf)} dop_buf={len(doppler_buf)}{dop_status} "
                          f"resets={len(reset_intervals)}",
                          file=sys.stderr, flush=True, end="")
                t_next_stats = now + stats_interval_s

            if tp != "iq":
                continue

            t = time.monotonic()
            # Skip records inside reboot guard windows
            in_reset = False
            for lo, hi in reset_intervals[-5:]:  # only check recent intervals
                if lo <= t <= hi:
                    in_reset = True
                    break
            if in_reset:
                continue

            rec["_t_rel"] = t
            buf.append(rec)
            doppler_buf.append(rec)
            if first_t is None:
                first_t = t
                next_emit_t = first_t + window_s
            # Trim short buffer to window_s of trailing data.
            bound_short = t - window_s
            while buf and float(buf[0].get("_t_rel", 0.0)) < bound_short:
                buf.popleft()
            # Trim long Doppler buffer to breathing_window_s of trailing data.
            bound_long = t - breathing_window_s
            while doppler_buf and float(doppler_buf[0].get("_t_rel", 0.0)) < bound_long:
                doppler_buf.popleft()
            if next_emit_t is None or t < next_emit_t:
                continue
            if not buf:
                next_emit_t = t + stride_s
                continue
            cur_amps = per_link_subcar_amps(list(buf))
            deltas = {}
            for link, base_v in baseline.items():
                if link in cur_amps and cur_amps[link].shape == base_v.shape:
                    deltas[link] = cur_amps[link] - base_v
            # Calibrationless mode (no baseline) -- still emit so Doppler
            # results reach the user. We just skip v2 / variance-gate /
            # template-amp_delta paths that need baseline-relative deltas.
            no_baseline_mode = (not baseline)
            if deltas or no_baseline_mode:
                # Empty detection by ACROSS-SUBCARRIER variance per link:
                # for each link, std of the 192-element delta vector
                # measures frequency-selective fading. Empty room has
                # uniform per-subcarrier delta (drift is gain-shift =
                # additive constant, leaves std unchanged). A body
                # creates frequency-selective fading: some subcarriers
                # drop, some rise -> std rises sharply. Take max across
                # links so any single link with body-induced subcarrier
                # structure trips the gate. No history needed.
                per_link_subcar_stds = {link: float(d.std())
                                        for link, d in deltas.items()}
                if per_link_subcar_stds and empty_threshold > 0:
                    max_subcar_std = max(per_link_subcar_stds.values())
                    is_empty_variance = max_subcar_std < empty_threshold
                else:
                    is_empty_variance = False
                    max_subcar_std = 0.0

                # Doppler-band presence detection on the LONG buffer.
                # Independent of variance gate. Computes per-(link, sc)
                # breathing-band power on a 90 s rolling buffer for
                # sharp FFT resolution (~0.011 Hz bins). Warm-up: while
                # doppler_buf has < BREATHING_MIN_DURATION_S of data,
                # don't score (treat as "not enough data").
                breathing_score = 0.0
                doppler_ready = False
                doppler = None
                robust_features = None
                bodies = None
                doppler_explanation = None
                # Submit a fresh compute request to the worker (snapshot
                # the doppler buffer) and pull whatever results have
                # finished computing since last emission. This NEVER
                # blocks: receive continues during the worker's heavy
                # FFT/SVD operations on another thread.
                if worker is not None and len(doppler_buf) > 0:
                    next_request_id += 1
                    worker.submit({
                        "request_id": next_request_id,
                        "t": t,
                        "records": list(doppler_buf),
                        "opts": {
                            "explain_doppler": explain_doppler,
                            "robust_doppler": robust_doppler,
                            "localize": localize,
                            "breathing_presence": breathing_presence,
                            "breathing_ratio_threshold": breathing_ratio_threshold,
                            "svd_drift_modes": svd_drift_modes,
                            "body_snr_threshold": body_snr_threshold,
                            "single_body": single_body,
                        },
                    })
                    for result in worker.drain_results():
                        if "error" not in result:
                            latest_dop_result = result
                if latest_dop_result is not None:
                    doppler_ready = latest_dop_result.get(
                        "doppler_ready", False
                    )
                    doppler = latest_dop_result.get("doppler")
                    breathing_score = latest_dop_result.get(
                        "breathing_score", 0.0
                    )
                    robust_features = latest_dop_result.get("robust_features")
                    bodies = latest_dop_result.get("bodies")
                    doppler_explanation = latest_dop_result.get(
                        "doppler_explain"
                    )
                if breathing_presence > 0 and doppler_ready:
                    is_empty_breathing = breathing_score < breathing_presence
                else:
                    # Doppler disabled or not warmed up yet -> don't gate
                    is_empty_breathing = True

                # Combined empty decision: the room is empty if all
                # ENABLED+READY gates agree. Doppler must be warmed up
                # (>= BREATHING_MIN_DURATION_S) to count.
                breathing_active = (breathing_presence > 0) and doppler_ready
                variance_active = (empty_threshold > 0)
                if breathing_active and variance_active:
                    is_empty = is_empty_variance and is_empty_breathing
                elif breathing_active:
                    is_empty = is_empty_breathing
                elif variance_active:
                    is_empty = is_empty_variance
                else:
                    is_empty = False

                # Rolling baseline: update only when room is empty (no
                # body-induced subcarrier structure detected). Drift will
                # accumulate uniformly so EMA tracks it; body presence
                # holds baseline fixed.
                if baseline_alpha > 0 and is_empty:
                    for link, cur_v in cur_amps.items():
                        if link in baseline and cur_v.shape == baseline[link].shape:
                            baseline[link] = ((1.0 - baseline_alpha) * baseline[link]
                                              + baseline_alpha * cur_v)

                # If breathing gate was requested but Doppler isn't yet
                # warmed up, suppress the emission entirely. The user
                # asked for "wait until full Doppler is ready before
                # making any guess", so we don't yield a class until the
                # 90s buffer has at least BREATHING_MIN_DURATION_S of
                # data. The variance gate alone could call EMPTY/quad
                # but the user's design requires Doppler corroboration.
                if breathing_presence > 0 and not doppler_ready:
                    if debug_log_fp is not None:
                        debug_log_fp.write(json.dumps({
                            "type": "warmup",
                            "t_wall": time.time(),
                            "n_records_doppler_buf": len(doppler_buf),
                            "doppler_buf_seconds": round(
                                float(doppler_buf[-1].get("_t_rel", 0.0))
                                - float(doppler_buf[0].get("_t_rel", 0.0)) if doppler_buf else 0.0,
                                2,
                            ),
                            "warmup_target_s": BREATHING_MIN_DURATION_S,
                        }) + "\n")
                        debug_log_fp.flush()
                    next_emit_t = t + stride_s
                    continue

                # Calibrationless Doppler-only pipeline: the headline
                # prediction comes from the continuous estimator + HMM
                # smoother computed below. The short-window class field
                # is a placeholder for compatibility with the emission
                # schema.
                pred, rule_idx = "DOPPLER_ONLY", 0
                if is_empty:
                    pred = "EMPTY"
                # Operator reset signal: clear HMM posterior + position
                # smoother + starvation streaks. Fires once per
                # 'r' + Enter (stdin) OR 'r' keypress in pygame window.
                if pygame_display is not None and pygame_display.reset_event.is_set():
                    pygame_display.reset_event.clear()
                    reset_request.set()
                if pygame_display is not None and pygame_display.stop_event.is_set():
                    stop_event.set()
                if reset_request.is_set():
                    reset_request.clear()
                    # Smoother + downstream state.
                    if hasattr(dop_smoother, "reset"):
                        dop_smoother.reset()
                    if pos_smoother is not None:
                        pos_smoother.reset()
                    for link in link_starve_streak:
                        link_starve_streak[link] = 0
                    last_starve_warning_t = 0.0
                    # Upstream IQ state: without this, the HMM is
                    # reset to uniform but the next window's per-link
                    # signal scores are still computed from a
                    # doppler_buf that is up to ~90 s of evidence from
                    # the OLD position. The HMM converges right back.
                    # Clearing the buffers and the multi-window
                    # accumulator forces post-reset evidence only.
                    buf.clear()
                    doppler_buf.clear()
                    if worker is not None:
                        worker.reset()
                    latest_dop_result = None
                    print(">>> RESET: HMM, position smoother, "
                          "starvation streaks, IQ buffers, and "
                          "Doppler accumulator cleared. Display will "
                          "show ANALYZING for ~30-90s while the new "
                          "breathing window fills.", flush=True)

                # Compute per-link AND per-node record counts over the
                # short window (window_s seconds). Per-node count = how
                # many IQ records the receiver-side decoder saw with
                # that node_id over the window, regardless of which
                # peer the record came from. This is the same data the
                # operator console prints as `[n1=.. n2=.. n3=.. n4=..]`
                # and is what feeds the pygame RX-rate panel.
                per_link_counts = {}
                per_node_counts = {1: 0, 2: 0, 3: 0, 4: 0}
                for r in buf:
                    nid, pid = r.get("node_id"), r.get("peer_id")
                    if isinstance(nid, int) and 1 <= nid <= 4:
                        per_node_counts[nid] += 1
                    if isinstance(nid, int) and isinstance(pid, int) and nid != pid:
                        a, b = sorted((nid, pid))
                        if 1 <= a and b <= 4:
                            k = f"{a}-{b}"
                            per_link_counts[k] = per_link_counts.get(k, 0) + 1

                # Update per-link starvation streaks. We use the LONG
                # (breathing) window's per-link packet count from the
                # doppler_explanation -- same source as
                # dominant_node_from_doppler / format_doppler_explanation
                # so the streak is consistent with the live "[doppler]
                # excluded ... starved link(s)" message. The SHORT 6s
                # window count (per_link_counts above) is too small to
                # apply a 70-pkt absolute floor to.
                # A link is starved if its packet count is below
                # max(starvation_min_packets, starvation_pct_of_median *
                # median). Sustained starvation = SoftAP queue is biased.
                _explain_pl = (doppler_explanation.get("per_link") or {}
                               if doppler_explanation else {})
                if _explain_pl:
                    pkt_vals = [info.get("n_packets", 0)
                                for info in _explain_pl.values()]
                    median_pkts = (sorted(pkt_vals)[len(pkt_vals) // 2]
                                    if pkt_vals else 0)
                    starve_cutoff = max(starvation_min_packets,
                                          starvation_pct_of_median * median_pkts)
                    for link in link_starve_streak:
                        a, b = link.split("-")
                        info = _explain_pl.get((int(a), int(b)))
                        if info is None:
                            continue
                        n = info.get("n_packets", 0)
                        if n < starve_cutoff:
                            link_starve_streak[link] += 1
                        else:
                            link_starve_streak[link] = 0
                elif per_link_counts:
                    # Fallback (no doppler_explanation yet): mark
                    # nothing as starved during warm-up so the warning
                    # banner doesn't fire spuriously.
                    pass
                    # Emit recommendation when any streak crosses the
                    # threshold. Rate-limit to once per
                    # starvation_warning_interval_s seconds so we don't
                    # spam stderr.
                    max_streak_link = max(link_starve_streak,
                                           key=link_starve_streak.get)
                    max_streak = link_starve_streak[max_streak_link]
                    now_t = time.time()
                    if (max_streak >= starvation_warn_threshold
                            and (now_t - last_starve_warning_t)
                                >= starvation_warning_interval_s):
                        last_starve_warning_t = now_t
                        warning_line = (
                            f">>> RECOMMENDATION: restart coord. "
                            f"L{max_streak_link.replace('-', '')} starved "
                            f"for {max_streak} consecutive windows "
                            f"(~{max_streak * stride_s:.0f}s). "
                            f"SoftAP queue likely biased; predictions "
                            f"will drift soon."
                        )
                        if not quiet:
                            print(warning_line, file=sys.stderr,
                                  flush=True)
                        # Also persist to stdout so it appears in the
                        # demo viewer log alongside other lines.
                        print(warning_line, flush=True)
                # These are populated whether or not debug_log_fp is
                # set, because the live_iter yield exposes them to the
                # udp dispatch (for ASCII headline + pygame display).
                dop_pred_node = None
                dop_pred_class = None
                dop_peak_hz = None
                dop_active_frac = None
                smoothed_node = None
                smoothed_class = None
                smoothed_confidence = 0.0
                per_link_active_fraction = None
                per_link_peak_snr = None
                obs_node_energy = None
                if True:
                    # Per-link active_fraction / peak_snr captured here
                    # so the debug log can be replayed through the
                    # continuous estimator without raw IQ.
                    per_link_active_fraction = None
                    per_link_peak_snr = None
                    obs_node_energy = None
                    if doppler_explanation is not None:
                        overall = doppler_explanation.get("overall", {}) or {}
                        dop_peak_hz = overall.get("peak_freq_hz")
                        dop_active_frac = overall.get("active_fraction")
                        explain_pl = doppler_explanation.get("per_link") or {}
                        if explain_pl:
                            per_link_active_fraction = {
                                f"{a}-{b}": round(
                                    info.get("active_fraction", 0.0), 4)
                                for (a, b), info in explain_pl.items()
                            }
                        if continuous_estimator:
                            # Continuous attribution over the explain
                            # pipeline (active_fraction-weighted endpoints).
                            _norm_explain = (per_link_baseline_explain
                                              if per_link_normalize else None)
                            dom_new, energy_p1, _ = continuous_node_estimator(
                                explain_pl, reliability_exp=0.5,
                                min_confidence=0.30,
                                per_link_baseline=_norm_explain,
                                min_link_packets=min_link_packets,
                            )
                            if dom_new is not None:
                                dop_pred_node = dom_new
                                dop_pred_class = {1: "Q3", 2: "Q1",
                                                   3: "Q2", 4: "Q4"}.get(
                                                       dom_new)
                            obs_node_energy = energy_p1
                            # EMA update for next window's normalization.
                            # Update AFTER the call so the current window
                            # is normalized against past windows only.
                            if per_link_normalize and explain_pl:
                                from live.continuous_estimator import (
                                    per_link_signal_score
                                )
                                _a = per_link_normalize_alpha
                                for _link, _info in explain_pl.items():
                                    _score = per_link_signal_score(_info)
                                    _prev = per_link_baseline_explain.get(
                                        _link, _score
                                    )
                                    per_link_baseline_explain[_link] = (
                                        (1.0 - _a) * _prev + _a * _score
                                    )
                        else:
                            # Binary top-K tally branch (used when
                            # --continuous-estimator is off).
                            dom = dominant_node_from_doppler(
                                doppler_explanation
                            )
                            if dom is not None:
                                dop_pred_node = dom
                                dop_pred_class = {1: "Q3", 2: "Q1",
                                                   3: "Q2", 4: "Q4"}.get(dom)
                    # Capture per-link peak_snr from robust_features for
                    # debug log enrichment and continuous attribution.
                    if robust_features:
                        per_link_peak_snr = {
                            f"{a}-{b}": round(info.get("peak_snr", 0.0), 3)
                            for (a, b), info in robust_features.items()
                        }
                    if continuous_estimator and robust_features:
                        # Continuous attribution over robust_features
                        # (peak_snr-weighted endpoints).
                        _norm_robust = (per_link_baseline_robust
                                         if per_link_normalize else None)
                        _, energy_p2, _ = continuous_node_estimator(
                            robust_features, reliability_exp=0.5,
                            min_confidence=0.30,
                            per_link_baseline=_norm_robust,
                            min_link_packets=min_link_packets,
                        )
                        if obs_node_energy is None:
                            obs_node_energy = energy_p2
                        else:
                            for k, v in energy_p2.items():
                                obs_node_energy[k] = (
                                    obs_node_energy.get(k, 0.0) + v
                                )
                        # EMA update for next window.
                        if per_link_normalize and robust_features:
                            from live.continuous_estimator import (
                                per_link_signal_score
                            )
                            _a = per_link_normalize_alpha
                            for _link, _info in robust_features.items():
                                _score = per_link_signal_score(_info)
                                _prev = per_link_baseline_robust.get(
                                    _link, _score
                                )
                                per_link_baseline_robust[_link] = (
                                    (1.0 - _a) * _prev + _a * _score
                                )
                    if continuous_estimator and use_blockage_signal:
                        # Differential-starvation attribution: bodies in
                        # a link's Fresnel zone drop that link's packet
                        # rate while peers stay healthy. Endpoints of
                        # starved links get a positive contribution
                        # proportional to the starvation depth. Blended
                        # additively with the breathing-signal energy
                        # via blockage_weight (default 0.30 -> roughly
                        # 1:3 relative to a fully-active breathing link).
                        _explain_pl = (doppler_explanation.get("per_link")
                                        or {} if doppler_explanation else {})
                        if _explain_pl:
                            _link_pkts = {
                                lk: int(info.get("n_packets", 0) or 0)
                                for lk, info in _explain_pl.items()
                            }
                            _block_e = blockage_attribution(_link_pkts)
                            if obs_node_energy is None:
                                obs_node_energy = {n: 0.0
                                                    for n in (1, 2, 3, 4)}
                            for k, v in _block_e.items():
                                obs_node_energy[k] = (
                                    obs_node_energy.get(k, 0.0)
                                    + blockage_weight * v
                                )
                    if continuous_estimator:
                        # HMM observation: combined explain-pipeline +
                        # robust-pipeline node_energy. Pass None when
                        # both are empty so the smoother applies the
                        # transition prior only.
                        if (obs_node_energy is None
                                or sum(obs_node_energy.values()) <= 0):
                            obs_node_energy = None
                        sm_node, smoothed_confidence = dop_smoother.update(
                            obs_node_energy
                        )
                    else:
                        # Robust-pipeline body detection drives the
                        # headline prediction in the legacy branch.
                        body_dom_node = None
                        if bodies:
                            body_dom_node = bodies[0].get("dominant_node")
                        sm_node, smoothed_confidence = dop_smoother.update(
                            body_dom_node
                        )
                    if sm_node is not None:
                        smoothed_node = sm_node
                        smoothed_class = {1: "Q3", 2: "Q1",
                                            3: "Q2", 4: "Q4"}.get(
                            sm_node
                        )
                    # Per-body bodies count + dominant nodes
                    bodies_summary = None
                    if bodies:
                        bodies_summary = [
                            {
                                "bpm": round(b.get("bpm", 0.0), 1),
                                "snr": round(b.get("mean_peak_snr", 0.0), 2),
                                "dominant_node": b.get("dominant_node"),
                                "n_links": len(b.get("links", [])),
                            } for b in bodies
                        ]
                    if debug_log_fp is None:
                        pass  # skip debug log write
                    else:
                      debug_log_fp.write(json.dumps({
                        "type": "window",
                        "t_wall": time.time(),
                        "class": pred,
                        "rule_index": rule_idx,
                        "is_empty": is_empty,
                        "max_subcar_std": round(max_subcar_std, 4),
                        "breathing_score": round(breathing_score, 4),
                        "doppler_ready": doppler_ready,
                        "doppler_pred_node": dop_pred_node,
                        "doppler_pred_class": dop_pred_class,
                        "doppler_smoothed_node": smoothed_node,
                        "doppler_smoothed_class": smoothed_class,
                        "doppler_smoothed_confidence": round(
                            smoothed_confidence, 3
                        ),
                        "doppler_peak_bpm": (round(dop_peak_hz * 60.0, 1)
                                               if dop_peak_hz is not None
                                               else None),
                        "n_bodies": len(bodies) if bodies else 0,
                        "doppler_peak_hz": (round(dop_peak_hz, 3)
                                              if dop_peak_hz is not None
                                              else None),
                        "doppler_active_fraction": (
                            round(dop_active_frac, 4)
                            if dop_active_frac is not None else None
                        ),
                        "bodies": bodies_summary,
                        "n_records_buf": len(buf),
                        "n_records_doppler_buf": len(doppler_buf),
                        "per_link_record_counts": per_link_counts,
                        "per_link_subcar_std": {f"{a}-{b}": round(per_link_subcar_stds.get((a, b), 0.0), 4)
                                                 for (a, b) in ALL_LINKS},
                        "per_link_mean_delta": {f"{a}-{b}": round(float(deltas[(a, b)].mean()), 4)
                                                if (a, b) in deltas else None
                                                for (a, b) in ALL_LINKS},
                        "per_link_active_fraction": per_link_active_fraction,
                        "per_link_peak_snr": per_link_peak_snr,
                        "per_link_starve_streak": dict(link_starve_streak),
                        "n_recent_resets": len(reset_intervals),
                        "estimator_mode": ("continuous"
                                            if continuous_estimator
                                            else "binary_tally"),
                    }) + "\n")
                      debug_log_fp.flush()
                # doppler_explanation came from the worker (above).
                # 2D wall-pair localization. Operates on whatever data
                # is available -- delta-std alone is enough for motion;
                # Doppler SNR adds discrimination for stationary bodies.
                position = None
                per_body_positions = None
                if localize:
                    position = localizer_2d.estimate_position(
                        deltas,
                        robust_doppler=robust_features,
                    )
                    if position is not None and pos_smoother is not None:
                        px, py, pinfo = position
                        sx, sy = pos_smoother.update(px, py)
                        position = (sx, sy,
                                     {**pinfo, "raw_x": px, "raw_y": py,
                                      "ema_n": pos_smoother.n})
                    if bodies is not None and robust_features is not None:
                        per_body_positions = (
                            localizer_2d.estimate_position_per_body(
                                bodies, robust_features, deltas=deltas,
                            )
                        )
                yield {"class": pred, "rule_index": rule_idx,
                       "deltas": deltas, "t_wall": time.time(),
                       "n_records": len(buf), "is_empty": is_empty,
                       "max_subcar_std": max_subcar_std,
                       "doppler_explain": doppler_explanation,
                       "robust_features": robust_features,
                       "bodies": bodies,
                       "position": position,
                       "per_body_positions": per_body_positions,
                       "window_records": list(buf),
                       "doppler_smoothed_class": smoothed_class,
                       "doppler_smoothed_confidence": smoothed_confidence,
                       "doppler_pred_class": dop_pred_class,
                       "doppler_peak_bpm": (
                           round(dop_peak_hz * 60.0, 1)
                           if dop_peak_hz is not None else None
                       ),
                       "doppler_active_fraction": dop_active_frac,
                       "doppler_ready": doppler_ready,
                       "n_bodies": len(bodies) if bodies else 0,
                       "per_link_record_counts": per_link_counts,
                       "per_node_record_counts": dict(per_node_counts),
                       "window_seconds": window_s,
                       "per_link_starve_streak": dict(link_starve_streak),
                       "last_reset_info": (dict(last_reset_info)
                                            if last_reset_info else None),
                       "per_node_crash_count": dict(per_node_crash_count),
                       "n_total_resets": len(reset_intervals),
                       "recent_resets": list(recent_resets)}
            next_emit_t = t + stride_s
    finally:
        if worker is not None:
            worker.stop()
        rx.close()
        if not quiet:
            print("", file=sys.stderr, flush=True)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Live WiFi-CSI quadrant detector. "
                    "Continuous-attribution estimator + HMM smoother over the "
                    "4-state quadrant space.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_udp = sub.add_parser("udp",
                            help="Live UDP from coord, stream predictions.")
    p_udp.add_argument("--baseline", type=Path, default=None,
                        help="Path to empty-room baseline JSON. Required for "
                             "v2 rule output, variance gate, and amp_delta-"
                             "based template matching. Omit for calibrationless "
                             "Doppler+SVD mode (no baseline needed -- the SVD "
                             "denoise + breathing-band FFT works directly on "
                             "raw amplitudes).")
    p_udp.add_argument("--bind-host", default="0.0.0.0")
    p_udp.add_argument("--bind-port", type=int, default=4211)
    p_udp.add_argument("--window-seconds", type=float, default=WINDOW_SECONDS)
    p_udp.add_argument("--stride-seconds", type=float, default=ADVANCE_SECONDS)
    p_udp.add_argument("--smooth", type=int, default=5,
                        help="Majority-vote over last N predictions (default 5).")
    p_udp.add_argument("--quiet", action="store_true",
                        help="Suppress the [live] packet counter on stderr.")
    p_udp.add_argument("--empty-threshold", type=float, default=0.0,
                        help="ACROSS-SUBCARRIER variance threshold: in "
                             "each window, compute std across the 192-element "
                             "per-(link, subcarrier) delta vector for each "
                             "link, take max across links. If max < this, "
                             "output EMPTY. Empty rooms have uniform deltas "
                             "(low std). Body creates frequency-selective "
                             "fading (high std). Drift is uniform = doesn't "
                             "affect std. Try 1.5-2.5 to start.")
    p_udp.add_argument("--baseline-alpha", type=float, default=0.0,
                        help="Rolling-baseline EMA factor (0=fixed baseline). "
                             "When --empty-threshold > 0 and the current window "
                             "is detected as empty, the baseline updates by "
                             "alpha * current + (1-alpha) * baseline, tracking "
                             "mesh/room drift over time. Try 0.02 for a 3-hour "
                             "session (~50-window time constant ~50s settling).")
    p_udp.add_argument("--debug-log", type=Path, default=None,
                        help="Write per-window diagnostics + reset events as "
                             "JSONL to this path. Each line is one event with "
                             "n_records, per-link counts, per-link subcarrier "
                             "std, mean-delta, and prediction. Reset events "
                             "include node_id and reason. Useful for offline "
                             "investigation of packet drops and mesh stability.")
    p_udp.add_argument("--breathing-presence", type=float, default=0.0,
                        help="Doppler-based presence detection: fraction of "
                             "subcarriers showing breathing-band power above "
                             "noise floor that constitutes 'occupied'. Try "
                             "0.05 (=5%% of subcarriers). 0=disable.")
    p_udp.add_argument("--breathing-ratio-threshold", type=float,
                        default=BREATHING_BAND_RATIO_THRESHOLD,
                        help=f"Per-(link,sc) ratio threshold for considering "
                             f"a subcarrier 'breathing-active' (fraction of "
                             f"spectral power in 0.15-0.40 Hz band). Default "
                             f"{BREATHING_BAND_RATIO_THRESHOLD}.")
    p_udp.add_argument("--breathing-window-sec", type=float,
                        default=BREATHING_WINDOW_SECONDS,
                        help=f"Window for the Doppler/breathing FFT in "
                             f"seconds. Default {BREATHING_WINDOW_SECONDS} "
                             f"(FFT resolution 1/window, so 90 s -> 0.011 "
                             f"Hz bins, ~24 bins in the breathing band, "
                             f"sharp peak-vs-noise discrimination). The "
                             f"variance gate / classifier still uses the "
                             f"short --window-seconds. Doppler scoring is "
                             f"skipped until at least "
                             f"{BREATHING_MIN_DURATION_S:.0f}s of data is "
                             f"buffered (warm-up).")
    p_udp.add_argument("--explain-doppler", action="store_true",
                        help="Print self-explanatory Doppler analysis after "
                             "each prediction: peak breathing frequency, BPM, "
                             "active subcarriers, top breathing-active links. "
                             "Surfaces the physics, not just the verdict.")
    p_udp.add_argument("--robust-doppler", action="store_true",
                        help="Use Welch + Butterworth high-pass + 6-window "
                             "coherent integration Doppler features and emit "
                             "per-body breathing detections (frequency-"
                             "clustered links). Multi-body capable.")
    p_udp.add_argument("--localize", action="store_true",
                        help="Compute continuous (x, y) position via wall-"
                             "pair perturbation ratios. Emits a 2D estimate "
                             "alongside the v2 quadrant verdict.")
    p_udp.add_argument("--ascii-map", action="store_true",
                        help="Render a small ASCII room map after each "
                             "prediction. Requires --localize.")
    p_udp.add_argument("--position-smooth-alpha", type=float, default=0.15,
                        help="EMA factor for the (x, y) position estimate. "
                             "Stationary bodies should give stable position; "
                             "per-window noise needs smoothing. Try 0.1-0.2 "
                             "(corresponds to ~5-10 window time constant). "
                             "0=disable.")
    p_udp.add_argument("--svd-drift-modes", type=int, default=0,
                        help="Apply CARM-style SVD drift removal in the live "
                             "Doppler pipeline. K=1 drops dominant temporal "
                             "mode (drift). Calibrationless: no baseline or "
                             "template needed -- pure physics (SVD-cleaned "
                             "amplitudes -> Doppler -> dominant breathing-"
                             "active node = quadrant).")
    p_udp.add_argument("--single-body", action="store_true",
                        help="Suppress all but the highest-SNR breathing "
                             "body in the doppler-robust output. For "
                             "single-occupant demos where a single body's "
                             "breathing peak can fragment across links due "
                             "to FFT jitter or harmonics, looking like 2+ "
                             "bodies. Always combined with shared-node "
                             "merge.")
    p_udp.add_argument("--continuous-estimator", action="store_true",
                        default=True,
                        help="(default ON) Continuous-attribution "
                             "estimator (soft endpoint attribution + "
                             "sqrt(N) reliability weight) plus the HMM "
                             "forward-filter smoother. This is the only "
                             "supported estimator post the rule-classifier "
                             "cleanup; the flag is kept for backwards "
                             "compatibility.")
    p_udp.add_argument("--no-continuous-estimator",
                        dest="continuous_estimator",
                        action="store_false",
                        help="Disable the continuous estimator. Without "
                             "it the headline will stay 'warming up' "
                             "indefinitely unless a body is detected by "
                             "the legacy body-finder pipeline.")
    p_udp.add_argument("--hmm-p-stay", type=float, default=0.95,
                        help="HMM transition self-probability. Higher = "
                             "smoother, more confident, slower to switch. "
                             "0.95 -> ~20-window smoothing horizon. Try "
                             "0.90 for quicker response, 0.98 for very "
                             "stable predictions.")
    p_udp.add_argument("--hmm-min-confidence", type=float, default=0.50,
                        help="HMM posterior confidence threshold below "
                             "which the smoother emits None (ANALYZING). "
                             "0.50 = top quadrant has at least 50%% of "
                             "the posterior mass.")
    p_udp.add_argument("--hmm-adjacency-weight", type=float, default=4.0,
                        help="Kinematic constraint on HMM transitions. "
                             "1.0 = uniform (any non-self state equally "
                             "likely). >1.0 = wall-adjacent jumps "
                             "preferred over diagonal-opposite jumps. "
                             "4.0 (default) makes Q1->Q3 (adjacent) 4x "
                             "more likely per window than Q1->Q4 "
                             "(diagonal). Suppresses single-window "
                             "noise-driven leaps across the room.")
    p_udp.add_argument("--position-max-step-m", type=float, default=1.5,
                        help="Kinematic clamp on (x,y) position updates. "
                             "Raw localizer outputs farther than this "
                             "from the previous smoothed position get "
                             "pulled to the boundary along the line of "
                             "travel. 1.5 m matches a brisk walk at "
                             "1.5 m/s with 1s windows. 0 = disable.")
    p_udp.add_argument("--starvation-warn-threshold", type=int, default=30,
                        help="Number of consecutive windows a link can "
                             "be excluded as 'starved' before the system "
                             "recommends restarting the coord. With "
                             "stride=2s, 30 windows = ~60s sustained "
                             "starvation.")
    p_udp.add_argument("--starvation-warning-interval-s", type=float,
                        default=20.0,
                        help="Minimum seconds between repeated coord-"
                             "restart recommendations. Default 20s -- "
                             "the warning persists, just doesn't spam.")
    p_udp.add_argument("--pygame", action="store_true",
                        help="Open the pygame demo display. Renders the "
                             "4-quadrant room map, body asterisk + "
                             "confidence circle, per-link breathing "
                             "bars, and operator banners. R = reset HMM, "
                             "Q/ESC = quit.")
    p_udp.add_argument("--pygame-fullscreen", action="store_true",
                        help="Open the pygame display in fullscreen.")
    p_udp.add_argument("--hide-body-marker", action="store_true",
                        help="Hide the (x, y) body asterisk + confidence "
                             "glow circle on the pygame display. The "
                             "quadrant headline and pulse remain. Useful "
                             "when the (x, y) layer is noisy and you "
                             "only want to demo the quadrant prediction.")
    p_udp.add_argument("--per-link-normalize", action="store_true",
                        help="Normalize each link's signal_score by its "
                             "rolling EMA baseline before soft endpoint "
                             "attribution. Each link contributes its "
                             "deviation from its own baseline rather than "
                             "its absolute strength. Use this when one "
                             "diagonal link is structurally stronger than "
                             "the other (e.g. SoftAP TXQ asymmetry biasing "
                             "predictions to one pair of quadrants).")
    p_udp.add_argument("--per-link-normalize-alpha", type=float, default=0.02,
                        help="EMA alpha for the per-link baseline. Smaller "
                             "= longer memory (~1/alpha windows). Default "
                             "0.02 -> ~50-window soft horizon (~50 s at "
                             "1 s stride). Only used when "
                             "--per-link-normalize is set.")
    p_udp.add_argument("--use-blockage-signal", action="store_true",
                        help="Use differential per-link starvation as a "
                             "positive position signal. A body in a link's "
                             "Fresnel zone drops that link's packet rate "
                             "while peers stay healthy; endpoints of "
                             "starved links get bonus node_energy. "
                             "Useful when breathing-band Doppler is weak "
                             "but the body is clearly blocking one link.")
    p_udp.add_argument("--blockage-weight", type=float, default=0.30,
                        help="Weight of the blockage signal relative to "
                             "the breathing-band energy. 0 disables "
                             "blockage; 0.3 (default) is moderate; 1.0 "
                             "makes blockage as strong as a fully-active "
                             "breathing link. Only used when "
                             "--use-blockage-signal is set.")
    p_udp.add_argument("--min-link-packets", type=int, default=0,
                        help="Drop links with fewer than this many "
                             "packets in the long (90 s) breathing "
                             "window from soft attribution entirely. "
                             "0 (default) = no filter. Use ~150 to "
                             "suppress chronically weak links whose "
                             "small-N denominator saturates "
                             "active_fraction with noise.")

    args = parser.parse_args(argv)

    if args.cmd == "udp":
        baseline = {}
        print(f"udp source: {args.bind_host}:{args.bind_port}", flush=True)
        print(f"waiting for first {args.window_seconds:.0f} s of packets "
              f"before first emission...", flush=True)
        if args.breathing_presence > 0:
            print(f"doppler/breathing gate enabled: window="
                  f"{args.breathing_window_sec:.0f}s, "
                  f"warm-up={BREATHING_MIN_DURATION_S:.0f}s "
                  f"(score=0 until warm-up complete)", flush=True)
        stop = threading.Event()
        smooth_buf = deque(maxlen=max(1, args.smooth))
        debug_log_fp = None
        if args.debug_log is not None:
            debug_log_fp = open(args.debug_log, "w", encoding="utf-8")
            print(f"writing debug log to {args.debug_log}", flush=True)

        # Optional pygame demo display
        pygame_disp = None
        if args.pygame:
            try:
                from live.pygame_display import PygameDisplay
                pygame_disp = PygameDisplay(
                    fullscreen=args.pygame_fullscreen,
                    show_body_marker=not args.hide_body_marker,
                )
                pygame_disp.start()
                print(f"[live] pygame display started "
                      f"(fullscreen={args.pygame_fullscreen}). "
                      f"Press R in the pygame window to reset HMM, "
                      f"Q/ESC to quit.", flush=True)
            except Exception as e:
                print(f"[live] WARNING: pygame display failed to start: "
                      f"{e}. Continuing in ASCII-only mode.",
                      file=sys.stderr, flush=True)
                pygame_disp = None
        try:
            for emission in live_iter(
                baseline,
                bind_host=args.bind_host,
                bind_port=args.bind_port,
                window_s=args.window_seconds,
                stride_s=args.stride_seconds,
                breathing_window_s=args.breathing_window_sec,
                stop_event=stop,
                quiet=args.quiet,
                empty_threshold=args.empty_threshold,
                baseline_alpha=args.baseline_alpha,
                debug_log_fp=debug_log_fp,
                breathing_presence=args.breathing_presence,
                breathing_ratio_threshold=args.breathing_ratio_threshold,
                explain_doppler=args.explain_doppler,
                robust_doppler=args.robust_doppler,
                localize=args.localize or args.ascii_map,
                position_smooth_alpha=args.position_smooth_alpha,
                svd_drift_modes=args.svd_drift_modes,
                single_body=args.single_body,
                continuous_estimator=args.continuous_estimator,
                hmm_p_stay=args.hmm_p_stay,
                hmm_min_confidence=args.hmm_min_confidence,
                hmm_adjacency_weight=args.hmm_adjacency_weight,
                position_max_step_m=args.position_max_step_m,
                starvation_warn_threshold=args.starvation_warn_threshold,
                starvation_warning_interval_s=(
                    args.starvation_warning_interval_s
                ),
                per_link_normalize=args.per_link_normalize,
                per_link_normalize_alpha=args.per_link_normalize_alpha,
                use_blockage_signal=args.use_blockage_signal,
                blockage_weight=args.blockage_weight,
                min_link_packets=args.min_link_packets,
                pygame_display=pygame_disp,
            ):
                smooth_buf.append(emission["class"])
                counts = {}
                for c in smooth_buf:
                    counts[c] = counts.get(c, 0) + 1
                smoothed = max(counts, key=counts.get)
                print(format_prediction(smoothed,
                                         emission["deltas"],
                                         ts_wall=emission["t_wall"],
                                         n_records=emission["n_records"],
                                         rule_index=emission.get("rule_index")),
                      flush=True)
                # If --explain-doppler is enabled, print the physics
                # summary after each prediction.
                # === HEADLINE: smoothed quadrant prediction ===
                # The displayed verdict consumed by the live console
                # output and the pygame visualizer.
                sm_class = emission.get("doppler_smoothed_class")
                sm_conf = emission.get("doppler_smoothed_confidence", 0.0)
                ts_str = time.strftime("%H:%M:%S")
                if sm_class:
                    headline = (f"=== [{ts_str}] PREDICTION: {sm_class} "
                                f"({sm_conf*100:.0f}% confidence) ===")
                elif sm_conf > 0:
                    headline = (f"=== [{ts_str}] PREDICTION: ANALYZING "
                                f"({sm_conf*100:.0f}% conf, below 50% "
                                f"threshold) ===")
                else:
                    headline = f"=== [{ts_str}] PREDICTION: warming up ==="
                print(headline, flush=True)
                # Push everything we just computed to the pygame display
                if pygame_disp is not None:
                    _emis_pos = emission.get("position")
                    _pos_xy = (_emis_pos[0], _emis_pos[1]) \
                        if _emis_pos else None
                    _exp = emission.get("doppler_explain") or {}
                    _per_link_af = None
                    if _exp.get("per_link"):
                        _per_link_af = {
                            f"{a}-{b}": info.get("active_fraction", 0.0)
                            for (a, b), info in _exp["per_link"].items()
                        }
                    _starv = emission.get("per_link_starve_streak")
                    _pkts = emission.get("per_link_record_counts")
                    _bpm = emission.get("doppler_peak_bpm")
                    _af_overall = emission.get("doppler_active_fraction")
                    _starv_warn = False
                    if _starv:
                        _starv_warn = max(_starv.values()) >= 30
                    pygame_disp.update(
                        smoothed_class=sm_class,
                        confidence=sm_conf,
                        raw_class=emission.get("doppler_pred_class"),
                        bpm=_bpm,
                        active_fraction=_af_overall,
                        per_link_active_fraction=_per_link_af,
                        per_link_record_counts=_pkts,
                        per_link_starve_streak=_starv,
                        position_xy=_pos_xy,
                        n_bodies=emission.get("n_bodies", 0),
                        doppler_ready=emission.get("doppler_ready", False),
                        starvation_warning_active=_starv_warn,
                        n_records_short=emission.get("n_records"),
                        per_node_record_counts=emission.get(
                            "per_node_record_counts"
                        ),
                        window_seconds=emission.get("window_seconds"),
                        last_reset_info=emission.get("last_reset_info"),
                        per_node_crash_count=emission.get(
                            "per_node_crash_count"
                        ),
                        n_total_resets=emission.get("n_total_resets"),
                        recent_resets=emission.get("recent_resets"),
                    )
                    if pygame_disp.stop_event.is_set():
                        stop.set()
                        break
                exp = emission.get("doppler_explain")
                if exp:
                    print(format_doppler_explanation(
                        exp,
                        use_continuous=args.continuous_estimator,
                    ), flush=True)
                bodies = emission.get("bodies")
                robust_features = emission.get("robust_features")
                if bodies is not None and robust_features is not None:
                    print(format_robust_doppler(robust_features, bodies),
                          flush=True)
                position = emission.get("position")
                if position is not None:
                    px, py, pinfo = position
                    print(localizer_2d.format_position(px, py, pinfo),
                          flush=True)
                    if args.ascii_map:
                        print(localizer_2d.render_ascii_room(px, py),
                              flush=True)
                per_body = emission.get("per_body_positions")
                if per_body:
                    print(localizer_2d.format_per_body_positions(per_body),
                          flush=True)
                    if args.ascii_map:
                        print(
                            localizer_2d.render_ascii_room_multi(per_body),
                            flush=True,
                        )
        except KeyboardInterrupt:
            print("\nshutdown requested", flush=True)
            stop.set()
        finally:
            if debug_log_fp is not None:
                debug_log_fp.close()
            if pygame_disp is not None:
                pygame_disp.stop()
        return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
