"""Continuous-attribution node estimator + HMM smoother.

Per-window quadrant prediction from per-link Doppler signal scores.
No quadrant-specific thresholds, no hand-tuned rules.

    1. Soft endpoint attribution
       Each link L=(a,b) contributes 0.5 * signal_score(L) to node a
       and 0.5 * signal_score(L) to node b. A body that modulates
       only a single link still contributes to its two endpoints
       and so survives into the smoother as evidence.

    2. Variance-stabilized reliability weight
       weight(L) = (n_packets(L) / max_n_packets) ** 0.5

       Welch periodogram variance scales as 1/N, so the standard
       variance-stabilizing weight scales as sqrt(N). Starved links
       contribute proportionally less rather than being dropped at
       a hard cutoff.

    3. HMM forward-filter smoother
       4-state forward filter over {Q1, Q2, Q3, Q4}. Per-window
       observation likelihood is proportional to the node-energy
       vector (continuous, not argmax). Single hyperparameter
       `p_stay` controls the smoothing horizon.

The estimator accepts either:
    - per-link records from `doppler_self_explain` (with active_fraction,
      n_packets), OR
    - per-link records from `per_link_doppler_robust` (with peak_snr,
      n_packets_avg).

It synthesizes a unified per-link `signal_score` from whichever fields
are present.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Tuple

import numpy as np


_NODE_TO_CLASS = {1: "Q3", 2: "Q1", 3: "Q2", 4: "Q4"}
_NODES = (1, 2, 3, 4)

# Quadrant adjacency in our 4-corner layout. Two quadrants are
# 'adjacent' if they share a wall, 'diagonal' if they share only the
# room center. Q1=NW=node2, Q2=NE=node3, Q3=SW=node1, Q4=SE=node4.
#   Q1 adjacent to {Q2 (N wall), Q3 (W wall)}; diagonal {Q4}
#   Q2 adjacent to {Q1, Q4 (E wall)};            diagonal {Q3}
#   Q3 adjacent to {Q1, Q4 (S wall)};            diagonal {Q2}
#   Q4 adjacent to {Q2, Q3};                     diagonal {Q1}
# Indexed by NODE id (1..4), not Q-string.
_ADJACENT_NODES = {
    1: {2, 4},  # Q3 adjacent to Q1 (node 2), Q4 (node 4)
    2: {1, 3},  # Q1 adjacent to Q3 (node 1), Q2 (node 3)
    3: {2, 4},  # Q2 adjacent to Q1 (node 2), Q4 (node 4)
    4: {1, 3},  # Q4 adjacent to Q3 (node 1), Q2 (node 3)
}
_DIAGONAL_NODES = {
    1: {3},  # Q3 diagonal to Q2 (node 3)
    2: {4},  # Q1 diagonal to Q4 (node 4)
    3: {1},  # Q2 diagonal to Q3 (node 1)
    4: {2},  # Q4 diagonal to Q1 (node 2)
}


def per_link_n_packets(info: dict) -> int:
    """Return packet count from either explain (`n_packets`) or robust
    (`n_packets_avg`) record format."""
    n = info.get("n_packets")
    if n is None:
        n = info.get("n_packets_avg", 0)
    return int(n) if n is not None else 0


def per_link_signal_score(info: dict) -> float:
    """Continuous signal score from a per-link feature dict.

    Combines active_fraction (from `doppler_self_explain`) with
    peak_snr (from `per_link_doppler_robust`) into a single
    non-negative score representing the strength of breathing-band
    activity on this link.

    The two feature families have different scales:
      - active_fraction is a ratio in [0, 1] (typical 0-0.3)
      - peak_snr is a power ratio (typical 0-30)
    so when both are present we use their product (a strong AND
    wideband peak); when only one is present we fall back gracefully.
    """
    af = float(info.get("active_fraction", 0.0) or 0.0)
    snr = float(info.get("peak_snr", 0.0) or 0.0)
    if af > 0.0 and snr > 0.0:
        return af * snr
    if af > 0.0:
        return af
    if snr > 0.0:
        return snr * 0.05
    return 0.0


def continuous_node_estimator(per_link: Optional[Dict[Tuple[int, int], dict]],
                              reliability_exp: float = 0.5,
                              min_confidence: float = 0.30,
                              per_link_baseline: Optional[
                                  Dict[Tuple[int, int], float]] = None,
                              baseline_eps: float = 1e-3,
                              min_link_packets: int = 0,
                              ) -> Tuple[Optional[int], Dict[int, float], float]:
    """Soft endpoint attribution with variance-stabilized reliability.

    Args:
        per_link: dict mapping link tuple (a, b) -> per-link info dict.
        reliability_exp: exponent for the (n_pkts / n_max) weight.
            0.5 = Welch variance stabilization, 0.0 = no weighting,
            1.0 = aggressive packet-rate weighting.
        min_confidence: top_energy / sum_energy threshold below which
            the function returns None for the node (caller can choose
            to display ANALYZING / fall back to prior).
        per_link_baseline: optional rolling baseline of each link's
            recent mean signal_score. When provided, each link's
            contribution is divided by `baseline + baseline_eps` so
            the link contributes its DEVIATION from its own baseline
            rather than its absolute signal strength. This corrects
            for per-link rate or quality asymmetry (e.g. one diagonal
            running ~5x stronger than the other due to SoftAP TXQ
            bias). Pass None (default) to keep legacy behavior.
        baseline_eps: small constant added to baseline to avoid
            divide-by-zero on cold-start or fully-quiet links.
        min_link_packets: hard filter — links with fewer than this many
            packets in the long window contribute ZERO to soft
            attribution (skipped entirely). 0 = no filter (legacy).
            Use to suppress chronically-weak links whose small-N
            denominators saturate active_fraction with noise. A typical
            value for 90 s × 10 Hz nominal rate is ~150 (= half the
            expected count on a healthy link).

    Returns:
        (best_node, node_energy, confidence)

        best_node:    1..4 or None when below min_confidence
        node_energy:  dict {1:.., 2:.., 3:.., 4:..} (always populated)
        confidence:   top_energy / sum_energy in [0, 1]
    """
    node_energy = {n: 0.0 for n in _NODES}
    if not per_link:
        return None, node_energy, 0.0
    pkt_counts = [per_link_n_packets(info) for info in per_link.values()]
    n_max = max(pkt_counts) if pkt_counts else 0
    if n_max <= 0:
        return None, node_energy, 0.0
    for link, info in per_link.items():
        a, b = link
        if a not in _NODES or b not in _NODES:
            continue
        n = per_link_n_packets(info)
        # Hard filter: chronically-weak links saturate active_fraction
        # with noise (small denominator), so they're skipped entirely
        # when below `min_link_packets`. The default 0 disables the
        # filter; the soft sqrt(N) reliability still down-weights
        # weak links that pass the floor.
        if min_link_packets > 0 and n < min_link_packets:
            continue
        reliability = (n / n_max) ** reliability_exp
        raw = per_link_signal_score(info)
        if per_link_baseline is not None:
            base = float(per_link_baseline.get(link, 0.0)) + baseline_eps
            raw = raw / base
        s = raw * reliability
        node_energy[a] += 0.5 * s
        node_energy[b] += 0.5 * s
    total = sum(node_energy.values())
    if total <= 0.0:
        return None, node_energy, 0.0
    best = max(node_energy, key=node_energy.get)
    confidence = node_energy[best] / total
    if confidence < min_confidence:
        return None, node_energy, confidence
    return best, node_energy, confidence


def node_to_class(node: Optional[int]) -> Optional[str]:
    if node is None:
        return None
    return _NODE_TO_CLASS.get(node)


class HmmNodeSmoother:
    """Forward-filter (online HMM) smoother over the 4-node state space.

    States: 1=Q3, 2=Q1, 3=Q2, 4=Q4.

    Per-window the smoother:
      - applies a transition prior to last window's posterior
        (a missing observation produces a transition-only update),
      - multiplies by an observation likelihood proportional to the
        node-energy vector (continuous, not argmax), and
      - normalizes to obtain the new posterior.

    Single hyperparameter: `p_stay` (probability the body stays in
    the same quadrant from one window to the next). Default 0.95
    corresponds to a soft horizon ~1 / (1 - 0.95) = 20 windows.

    Usage:
        sm = HmmNodeSmoother(p_stay=0.95, min_confidence=0.50)
        for window in detector_loop:
            _, node_energy, _ = continuous_node_estimator(per_link)
            best_node, posterior_conf = sm.update(node_energy)
            ...
    """

    def __init__(self, p_stay: float = 0.95, min_confidence: float = 0.50,
                 obs_epsilon: float = 1e-3,
                 adjacency_weight: float = 1.0):
        """
        Args:
            p_stay: probability of staying in the same state.
            min_confidence: posterior threshold to commit a prediction.
            obs_epsilon: smoothing for observation likelihood.
            adjacency_weight: kinematic constraint. 1.0 = uniform
                transitions to any non-self state. >1.0 = bias
                toward wall-adjacent quadrants over diagonal-opposite
                quadrants. A person can step into an adjacent quadrant
                in one window but cannot teleport diagonally; setting
                adjacency_weight=4.0 makes adjacent-jump 4x more likely
                than diagonal-jump. Useful for suppressing single-window
                noise-driven flips across the room.
        """
        if not (0.0 < p_stay < 1.0):
            raise ValueError("p_stay must be in (0, 1)")
        if adjacency_weight <= 0:
            raise ValueError("adjacency_weight must be > 0")
        self.p_stay = p_stay
        self.min_confidence = min_confidence
        self.obs_epsilon = obs_epsilon
        self.adjacency_weight = adjacency_weight
        self._log_post = np.full(4, -math.log(4.0), dtype=np.float64)
        # Build proximity-aware transition matrix. For each row i:
        #   p[i, i]      = p_stay
        #   p[i, j adj]  ∝ adjacency_weight
        #   p[i, j diag] ∝ 1.0
        # Normalized so non-self mass = (1 - p_stay).
        self._log_trans = np.empty((4, 4), dtype=np.float64)
        for i in range(4):
            from_node = i + 1
            adj = _ADJACENT_NODES[from_node]
            diag = _DIAGONAL_NODES[from_node]
            # Two adjacent + one diagonal = 3 non-self states for each node
            denom = 2 * adjacency_weight + 1 * 1.0
            for j in range(4):
                to_node = j + 1
                if i == j:
                    self._log_trans[i, j] = math.log(p_stay)
                elif to_node in adj:
                    p = (1.0 - p_stay) * (adjacency_weight / denom)
                    self._log_trans[i, j] = math.log(p)
                elif to_node in diag:
                    p = (1.0 - p_stay) * (1.0 / denom)
                    self._log_trans[i, j] = math.log(p)
                else:
                    # Defensive (shouldn't happen with 4 nodes)
                    self._log_trans[i, j] = math.log(
                        (1.0 - p_stay) / 3.0
                    )
        # p_switch: uniform-transition average exposed for callers that
        # need a single non-self transition probability.
        self.p_switch = (1.0 - p_stay) / 3.0

    @staticmethod
    def _logsumexp(x: np.ndarray) -> float:
        m = float(np.max(x))
        if not math.isfinite(m):
            return m
        return m + math.log(float(np.sum(np.exp(x - m))))

    def update(self, node_energy: Optional[Dict[int, float]]
               ) -> Tuple[Optional[int], float]:
        """Push a per-window node-energy vector, return (best_node, conf).

        node_energy: dict {1:.., 2:.., 3:.., 4:..} (energies >= 0).
                     Pass None when no observation is available
                     (transition-only update).

        Returns:
            (best_node, posterior_confidence)

        best_node is 1..4 if posterior_confidence >= min_confidence,
        otherwise None (caller should display ANALYZING).
        """
        # Transition step: log_pred[j] = logsumexp_i(log_post[i] +
        # log_trans[i, j])
        log_pred = np.empty(4, dtype=np.float64)
        for j in range(4):
            log_pred[j] = self._logsumexp(self._log_post
                                            + self._log_trans[:, j])
        # Observation step: likelihood proportional to (energy + eps).
        # Adding eps avoids log(0) when one or more nodes have zero
        # energy in this window.
        if node_energy is not None:
            energies = np.array([node_energy.get(i + 1, 0.0)
                                  for i in range(4)],
                                  dtype=np.float64)
            total = float(energies.sum())
            if total > 0.0:
                eps = self.obs_epsilon * total
                likelihood = (energies + eps) / (total + 4.0 * eps)
                log_pred = log_pred + np.log(likelihood)
        # Normalize -> posterior
        log_norm = self._logsumexp(log_pred)
        if math.isfinite(log_norm):
            self._log_post = log_pred - log_norm
        else:
            self._log_post = np.full(4, -math.log(4.0), dtype=np.float64)
        best_idx = int(np.argmax(self._log_post))
        confidence = float(np.exp(self._log_post[best_idx]))
        if confidence < self.min_confidence:
            return None, confidence
        return best_idx + 1, confidence

    def reset(self):
        self._log_post = np.full(4, -math.log(4.0), dtype=np.float64)

    @property
    def posterior(self) -> Dict[int, float]:
        """Current posterior P(state | observations) as a dict."""
        p = np.exp(self._log_post)
        return {i + 1: float(p[i]) for i in range(4)}


def blockage_attribution(per_link_packet_counts: Optional[
                            Dict[Tuple[int, int], int]]
                          ) -> Dict[int, float]:
    """Soft endpoint attribution from per-link DIFFERENTIAL starvation.

    A body in a link's Fresnel zone attenuates and drops packets on
    that link without affecting unrelated links. The asymmetry
    "this one link is starved while peers are healthy" is positive
    evidence that the body is on/near that link's line of sight.
    Endpoints of starved links get bonus energy.

    For each link, starvation_score = max(0, (median - this) / median)
    in [0, 1]. 0 = healthy or above-median; 1 = fully starved. Each
    link's score is split 50/50 across its two endpoint nodes.

    The output is intended to be ADDED to the breathing-signal-driven
    node_energy from continuous_node_estimator, weighted by a small
    blockage_weight that the caller chooses based on how reliable
    starvation is as a position signal in their environment (e.g.
    0.3 for moderate weight). When blockage_weight=0 the caller
    skips this entirely; when very high it can dominate the
    breathing signal — typically 0.2-0.5 is the right range.
    """
    energy = {n: 0.0 for n in _NODES}
    if not per_link_packet_counts:
        return energy
    counts = [int(c) for c in per_link_packet_counts.values() if c is not None]
    if not counts:
        return energy
    counts_sorted = sorted(counts)
    median_pkts = counts_sorted[len(counts_sorted) // 2]
    if median_pkts <= 0:
        return energy
    for link, n in per_link_packet_counts.items():
        a, b = link
        if a not in _NODES or b not in _NODES:
            continue
        if n is None:
            continue
        # Differential starvation: 0 = at-or-above median (healthy),
        # 1 = fully blocked. Body in Fresnel zone of one link drives
        # exactly one or two of the six toward 1 while peers stay
        # near 0 — that asymmetry is the signal.
        starvation = max(0.0, (median_pkts - int(n)) / float(median_pkts))
        energy[a] += 0.5 * starvation
        energy[b] += 0.5 * starvation
    return energy


__all__ = [
    "per_link_n_packets",
    "per_link_signal_score",
    "continuous_node_estimator",
    "node_to_class",
    "HmmNodeSmoother",
    "blockage_attribution",
]
