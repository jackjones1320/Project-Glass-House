"""2D wall-pair localizer.

Maps per-link perturbation strengths into a continuous (x, y) position
estimate in room coordinates, by ratioing perturbation on opposing
wall-edge links.

This is NOT triangulation in the time-of-flight sense -- our 40 MHz
HT40 bandwidth gives only c/(2*BW) = 3.75 m range resolution, useless
in a 9 m room. Instead, it is constraint-based: the 6 links between
4 corner nodes form 3 natural axis pairs:

    West edge L12 (n1=SW <-> n2=NW)  vs East edge L34 (n3=NE <-> n4=SE)
    South edge L14 (n1=SW <-> n4=SE) vs North edge L23 (n2=NW <-> n3=NE)
    Diagonal L13 (SW<->NE)           vs Diagonal L24 (NW<->SE)

The body lies somewhere on/near each link's line segment. A body close
to the west wall perturbs L12 strongly and L34 weakly -> the W/E
perturbation ratio gives x-coordinate. Same for y via S/N pair.

Diagonals are redundant with the wall pairs but useful for cross-
validation (hallway-adjacent links L24 and L34 may show inflated
perturbation from outside traffic; compare to L13 which is interior).

Per-link perturbation = per-link delta-vector std (frequency-selective
fading magnitude) + Doppler-band peak SNR (breathing modulation).
Either signal alone localizes; combining gives robustness when one
modality is weak (stationary body -> mostly Doppler; rapid motion ->
mostly std).

Geometry:
    Room: 3.81 m wide (W-E)  x  9.14 m long (S-N)
    Node 1 = Q3 = SW = (0.00, 0.00)
    Node 2 = Q1 = NW = (0.00, 9.14)
    Node 3 = Q2 = NE = (3.81, 9.14)
    Node 4 = Q4 = SE = (3.81, 0.00)
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

import numpy as np

ROOM_W = 3.81
ROOM_L = 9.14

NODE_POS = {
    1: (0.00, 0.00),
    2: (0.00, ROOM_L),
    3: (ROOM_W, ROOM_L),
    4: (ROOM_W, 0.00),
}

ALL_LINKS = ((1, 2), (1, 3), (1, 4), (2, 3), (2, 4), (3, 4))

WEST_LINK = (1, 2)
EAST_LINK = (3, 4)
SOUTH_LINK = (1, 4)
NORTH_LINK = (2, 3)
DIAG_SW_NE = (1, 3)
DIAG_NW_SE = (2, 4)

NODE_TO_CLASS = {1: "Q3", 2: "Q1", 3: "Q2", 4: "Q4"}


def _link_perturbation(deltas, robust_doppler, link, doppler_weight=0.5):
    """Combined std + Doppler-SNR perturbation for a single link.

    std-of-delta captures frequency-selective fading from any motion or
    blockage. Doppler peak-SNR captures breathing-band coherent power
    (works even when delta is zero because baseline is stale).

    The (snr - 1) clamp at zero acts as a noise-floor subtraction:
    peak_snr <= 1 means peak is at or below the median in-band power
    (no real peak), so contributes nothing.
    """
    s = 0.0
    if deltas is not None and link in deltas:
        s += float(deltas[link].std())
    if robust_doppler is not None and link in robust_doppler:
        snr = float(robust_doppler[link].get("peak_snr", 0.0))
        s += doppler_weight * max(0.0, snr - 1.0)
    return s


def compute_link_perturbations(deltas, robust_doppler=None,
                                doppler_weight=0.5):
    """Compute per-link perturbation strength dict."""
    return {link: _link_perturbation(deltas, robust_doppler, link,
                                       doppler_weight=doppler_weight)
            for link in ALL_LINKS}


def estimate_position(deltas, robust_doppler=None,
                      alpha=1.5,
                      doppler_weight=0.5,
                      min_total_perturbation=0.05):
    """Wall-pair 2D position estimate.

    For each axis (x: W-E, y: S-N) compute a sharpened ratio of
    perturbation on the two opposing wall links:

        x = ROOM_W * s_E^alpha / (s_W^alpha + s_E^alpha)
        y = ROOM_L * s_N^alpha / (s_S^alpha + s_N^alpha)

    `alpha` > 1 sharpens the response (closer to step function),
    alpha = 1 is pure linear ratio. alpha = 1.5 is a good compromise
    for the demo: smooth movement, but corners go to corners.

    `min_total_perturbation` is a noise gate: if both wall sums are
    near zero, return None (no body detected, position undefined).

    Returns (x, y, info) tuple or None.
    """
    perts = compute_link_perturbations(deltas, robust_doppler,
                                          doppler_weight=doppler_weight)

    s_w = perts[WEST_LINK]
    s_e = perts[EAST_LINK]
    s_s = perts[SOUTH_LINK]
    s_n = perts[NORTH_LINK]

    if (s_w + s_e) < min_total_perturbation and \
       (s_s + s_n) < min_total_perturbation:
        return None

    if (s_w + s_e) > 0:
        wa = s_w ** alpha
        ea = s_e ** alpha
        x = ROOM_W * ea / (wa + ea)
    else:
        x = ROOM_W / 2.0

    if (s_s + s_n) > 0:
        sa = s_s ** alpha
        na = s_n ** alpha
        y = ROOM_L * na / (sa + na)
    else:
        y = ROOM_L / 2.0

    diag_sw_ne = perts[DIAG_SW_NE]
    diag_nw_se = perts[DIAG_NW_SE]
    info = {
        "wall_perturbations": {
            "west_L12": s_w, "east_L34": s_e,
            "south_L14": s_s, "north_L23": s_n,
        },
        "diagonal_perturbations": {
            "L13_SW_NE": diag_sw_ne, "L24_NW_SE": diag_nw_se,
        },
        "link_perturbations": perts,
        "alpha": alpha,
    }
    return (x, y, info)


class PositionSmoother:
    """Exponential moving average for (x, y) position estimates.

    For STATIONARY bodies the per-window estimate is noisy because
    per-link std-of-delta has window-to-window variance even though
    the underlying body location is fixed. EMA averages out that
    variance with a configurable response time.

    alpha=0.1 -> ~20-window time constant (~20s at stride=1s, ~3 min
    at stride=10s). Tune for the demo: stationary subject -> low alpha
    (smoother). Moving subject -> higher alpha (faster response).
    """

    def __init__(self, alpha=0.15, max_step_m=None, stride_s=1.0):
        """
        Args:
            alpha: EMA blending factor (0=no update, 1=ignore history).
            max_step_m: kinematic constraint. If set, the per-update
                position can only move at most `max_step_m` meters from
                the previous estimate. Raw localizer outputs farther
                than that get pulled to the boundary along the line of
                travel. A person walking briskly at ~1.5 m/s with a
                stride_s=1s window can't move more than ~1.5 m per
                update; suppresses single-window jumps caused by
                upstream signal flips (SoftAP queue rotation, link
                starvation oscillation).
            stride_s: not used for EMA itself; recorded so callers can
                interpret the velocity bound (max_step_m / stride_s).
        """
        self.alpha = alpha
        self.max_step_m = max_step_m
        self.stride_s = stride_s
        self.x = None
        self.y = None
        self.n = 0

    def update(self, x, y):
        # Kinematic clamp BEFORE EMA: the raw input represents what the
        # localizer thinks the body is right now. If it's >max_step_m
        # from our last estimate, treat the input as if the body moved
        # exactly max_step_m in the requested direction. EMA still
        # applies after clamping so the estimate is dampened.
        if self.x is not None and self.max_step_m is not None:
            dx = x - self.x
            dy = y - self.y
            d = (dx * dx + dy * dy) ** 0.5
            if d > self.max_step_m and d > 0:
                scale = self.max_step_m / d
                x = self.x + dx * scale
                y = self.y + dy * scale
        if self.x is None:
            self.x = x
            self.y = y
        else:
            self.x = (1 - self.alpha) * self.x + self.alpha * x
            self.y = (1 - self.alpha) * self.y + self.alpha * y
        self.n += 1
        return (self.x, self.y)

    def reset(self):
        self.x = None
        self.y = None
        self.n = 0


def estimate_position_per_body(bodies, robust_doppler,
                                 deltas=None,
                                 doppler_weight=1.0,
                                 alpha=1.5):
    """One (x, y) per detected body.

    Each body has a frequency cluster and a list of links that show
    breathing at that frequency. We localize EACH body by computing
    wall-pair perturbation using ONLY the body's link list. Other
    links (assigned to different bodies, or quiet) contribute zero.

    This is the "differential body localization" idea: in multi-body
    rooms, the wall-pair ratio over ALL links averages every body's
    contribution. By restricting to one body's link cluster, we
    isolate that body's perturbation.

    Args:
        bodies: list of body dicts from detect_bodies_from_doppler.
        robust_doppler: dict[link] -> {peak_snr, peak_freq_hz, ...}.
        deltas: optional per-link delta vectors. If provided, std is
                added to the per-link perturbation, but only on
                body-cluster links.
        doppler_weight: weight on Doppler peak-snr vs delta-std.
                         Higher means trust Doppler more.

    Returns list of dicts:
        [{
            'body_index': 0,
            'frequency_hz': 0.25,
            'bpm': 15.0,
            'x': 1.5, 'y': 6.2,
            'links': [(1,2), (2,3)],
            'mean_peak_snr': 5.2,
        }, ...]
    """
    out = []
    for i, body in enumerate(bodies):
        body_links = set(body["links"])
        # Build per-link perturbation, zero outside the body's links
        per_link_pert = {}
        for link in ALL_LINKS:
            if link not in body_links:
                per_link_pert[link] = 0.0
                continue
            s = 0.0
            if deltas is not None and link in deltas:
                s += float(deltas[link].std())
            if robust_doppler is not None and link in robust_doppler:
                snr = float(robust_doppler[link].get("peak_snr", 0.0))
                s += doppler_weight * max(0.0, snr - 1.0)
            per_link_pert[link] = s

        s_w = per_link_pert[WEST_LINK]
        s_e = per_link_pert[EAST_LINK]
        s_s = per_link_pert[SOUTH_LINK]
        s_n = per_link_pert[NORTH_LINK]
        diag_sw_ne = per_link_pert[DIAG_SW_NE]
        diag_nw_se = per_link_pert[DIAG_NW_SE]

        # If a wall pair is empty (no body link on either side), fall
        # back to using the diagonals to fill in. Diagonal contains
        # both x and y info: L13 (SW-NE) bias means body is in either
        # SW or NE; L24 (NW-SE) bias means body is in either NW or SE.
        # Combined with whichever wall pair is non-zero, this gives
        # a unique quadrant.
        if (s_w + s_e) <= 0 and (diag_sw_ne + diag_nw_se) > 0:
            # Diagonal-only x: L13 high -> body on diagonal SW-NE
            # (could be x=0 or x=ROOM_W). L24 high -> NW-SE diagonal.
            # We use which y is suggested (below) to disambiguate.
            s_w_eff = diag_sw_ne if s_n > s_s else 0.0  # SW or NE
            s_e_eff = diag_nw_se if s_n > s_s else 0.0  # NW or SE
            # If unclear, distribute evenly
            s_w = s_w_eff if (s_w_eff + s_e_eff) > 0 else diag_nw_se
            s_e = s_e_eff if (s_w_eff + s_e_eff) > 0 else diag_sw_ne
        if (s_s + s_n) <= 0 and (diag_sw_ne + diag_nw_se) > 0:
            s_s_eff = diag_nw_se if s_e > s_w else diag_sw_ne
            s_n_eff = diag_sw_ne if s_e > s_w else diag_nw_se
            s_s = s_s_eff
            s_n = s_n_eff

        if (s_w + s_e) > 0:
            wa = s_w ** alpha
            ea = s_e ** alpha
            x = ROOM_W * ea / (wa + ea)
        else:
            x = ROOM_W / 2.0
        if (s_s + s_n) > 0:
            sa = s_s ** alpha
            na = s_n ** alpha
            y = ROOM_L * na / (sa + na)
        else:
            y = ROOM_L / 2.0

        out.append({
            "body_index": i,
            "frequency_hz": body["frequency_hz"],
            "bpm": body["bpm"],
            "x": x, "y": y,
            "links": body["links"],
            "mean_peak_snr": body["mean_peak_snr"],
            "dominant_node": body.get("dominant_node"),
        })
    return out


def format_per_body_positions(per_body):
    """Multi-line formatter for per-body localization output."""
    if not per_body:
        return "[per-body] no bodies detected"
    lines = []
    for body in per_body:
        link_str = ", ".join(f"L{a}{b}" for (a, b) in body["links"])
        q = quadrant_for_position(body["x"], body["y"])
        lines.append(
            f"[per-body] body {body['body_index']+1}: "
            f"{body['bpm']:.0f} BPM "
            f"(SNR {body['mean_peak_snr']:.1f}) "
            f"-> ({body['x']:.2f}m, {body['y']:.2f}m) {q}  "
            f"on {link_str}"
        )
    return "\n".join(lines)


def render_ascii_room_multi(per_body, width=21, height=15):
    """ASCII room with one digit per body (1, 2, ...) at its (x, y)."""
    grid = [[" " for _ in range(width)] for _ in range(height)]

    def to_grid(rx, ry):
        gx = int(round(rx / ROOM_W * (width - 1)))
        gy = int(round((1.0 - ry / ROOM_L) * (height - 1)))
        gx = max(0, min(width - 1, gx))
        gy = max(0, min(height - 1, gy))
        return gx, gy

    for n, (px, py) in NODE_POS.items():
        gx, gy = to_grid(px, py)
        grid[gy][gx] = str(n)

    for i, body in enumerate(per_body):
        gx, gy = to_grid(body["x"], body["y"])
        marker = chr(ord("a") + i) if i < 26 else "*"
        if grid[gy][gx] == " ":
            grid[gy][gx] = marker

    border = "+" + "-" * width + "+"
    rows = [border + "  N (top)"]
    for r in grid:
        rows.append("|" + "".join(r) + "|")
    rows.append(border)
    rows.append(f"  W <-- {ROOM_W:.2f}m x {ROOM_L:.2f}m --> E   "
                f"({len(per_body)} bodies)")
    return "\n".join(rows)


def quadrant_for_position(x, y):
    """Map (x, y) to its enclosing quadrant per the memory mapping."""
    if x < ROOM_W / 2:
        return "Q1" if y > ROOM_L / 2 else "Q3"
    return "Q2" if y > ROOM_L / 2 else "Q4"


def format_position(x, y, info=None):
    q = quadrant_for_position(x, y)
    out = f"[localizer] (x, y) = ({x:.2f}m, {y:.2f}m) -> {q}"
    if info is not None:
        wp = info["wall_perturbations"]
        out += (f"  W/E={wp['west_L12']:.2f}/{wp['east_L34']:.2f}  "
                f"S/N={wp['south_L14']:.2f}/{wp['north_L23']:.2f}")
    return out


def render_ascii_room(x, y, width=21, height=15, body_char="*"):
    """ASCII map: corners are node IDs 1..4, body is *.

    Coordinate convention: terminal row 0 = North (top of map);
    terminal column 0 = West (left of map). So we invert y for display.
    """
    grid = [[" " for _ in range(width)] for _ in range(height)]

    def to_grid(rx, ry):
        gx = int(round(rx / ROOM_W * (width - 1)))
        gy = int(round((1.0 - ry / ROOM_L) * (height - 1)))
        gx = max(0, min(width - 1, gx))
        gy = max(0, min(height - 1, gy))
        return gx, gy

    for n, (px, py) in NODE_POS.items():
        gx, gy = to_grid(px, py)
        grid[gy][gx] = str(n)

    if x is not None and y is not None:
        gx, gy = to_grid(x, y)
        if grid[gy][gx] in (" ", "."):
            grid[gy][gx] = body_char
        else:
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < width and 0 <= ny < height \
                       and grid[ny][nx] == " ":
                        grid[ny][nx] = body_char
                        break
                else:
                    continue
                break

    border = "+" + "-" * width + "+"
    rows = [border + "  N (top)"]
    for r in grid:
        rows.append("|" + "".join(r) + "|")
    rows.append(border)
    rows.append(f"  W <-- {ROOM_W:.2f}m x {ROOM_L:.2f}m --> E")
    return "\n".join(rows)
