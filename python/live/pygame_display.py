"""Pygame display for the deterministic-detector live UDP path.

Pure view layer: receives per-emission state via `update()`, renders a
4-quadrant room map with the current prediction lit up, a body asterisk
+ confidence circle, big headline text, per-link health bars, and the
starvation/reset operator banners.

Run as a thread alongside live_iter; the main event/render loop polls
the latest state. 'r' keypress sets a reset flag the detector watches.
'q' keypress exits.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

try:
    import pygame
except ImportError:  # graceful fallback when pygame isn't installed
    pygame = None


# Colors
_BG = (12, 12, 18)
_DARK = (40, 40, 48)
_LIT = (40, 200, 100)
_LIT_DIM = (30, 140, 70)
_BORDER = (90, 90, 110)
_TEXT = (220, 220, 220)
_TEXT_MUTED = (160, 160, 170)
_BAR_BG = (60, 60, 70)
_BAR_FG = (90, 170, 220)
_BAR_STARVED = (210, 80, 80)
_BANNER_WARN = (210, 50, 50)
_BANNER_RESET = (90, 170, 220)
_CIRCLE = (255, 220, 100)
_ASTERISK = (255, 255, 255)

# Quadrant -> compass cell. Q1=NW (node 2), Q2=NE (node 3), Q3=SW
# (node 1), Q4=SE (node 4). Cell coords are (col, row) where col=0
# is left and row=0 is top.
_QUAD_CELL = {
    "Q1": (0, 0),  # NW
    "Q2": (1, 0),  # NE
    "Q3": (0, 1),  # SW
    "Q4": (1, 1),  # SE
}

_LINKS_ORDERED = ["1-2", "1-3", "1-4", "2-3", "2-4", "3-4"]


class PygameDisplay:
    """Thread-safe pygame demo display.

    Lifecycle:
        d = PygameDisplay(width=1280, height=720)
        d.start()         # spawn render thread
        # ... in detector loop:
        d.update(emission)
        # ... at end:
        d.stop()

    Operator controls (rendered in the main pygame window):
        'r' or 'R' -> sets reset_event (read by the detector loop to
                       reset HMM + position smoother)
        'q' or ESC -> sets stop_event (detector should exit)
    """

    def __init__(self, width: int = 1280, height: int = 720,
                 fullscreen: bool = False,
                 room_width_m: float = 3.81,
                 room_length_m: float = 9.14,
                 show_body_marker: bool = True):
        if pygame is None:
            raise RuntimeError(
                "pygame is not installed. Run: pip install pygame-ce"
            )
        self.width = width
        self.height = height
        self.fullscreen = fullscreen
        self.room_width_m = room_width_m
        self.room_length_m = room_length_m
        self.show_body_marker = show_body_marker

        self._lock = threading.Lock()
        self._latest = {
            "smoothed_class": None,
            "confidence": 0.0,
            "raw_class": None,
            "bpm": None,
            "active_fraction": None,
            "per_link_active_fraction": None,
            "per_link_record_counts": None,
            "per_link_starve_streak": None,
            "position_xy": None,  # (x, y) in meters, room coords
            "n_bodies": 0,
            "doppler_ready": False,
            "starvation_warning_active": False,
            "last_reset_t": 0.0,
            # RX stats panel: total IQ records and per-node breakdown
            # over the past `window_seconds` of the live stream.
            "n_records_short": None,         # int total, e.g. 522
            "per_node_record_counts": None,  # {1: .., 2: .., 3: .., 4: ..}
            "window_seconds": None,          # float, e.g. 6.0
            # Last-reset panel
            "last_reset_info": None,         # {node_id, reason_label, crash_count, t_wall}
            "per_node_crash_count": None,    # {1: .., 2: .., 3: .., 4: ..}
            "n_total_resets": 0,             # int, since detector start
            "recent_resets": None,           # list of recent reset dicts
        }
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.reset_event = threading.Event()
        self.stop_event = threading.Event()

    def update(self, **kwargs):
        """Push the latest emission state. Thread-safe."""
        with self._lock:
            for k, v in kwargs.items():
                if k in self._latest:
                    self._latest[k] = v

    def trigger_reset_visual(self):
        with self._lock:
            self._latest["last_reset_t"] = time.time()

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self):
        pygame.init()
        if self.fullscreen:
            # On Windows, pygame.display.set_mode((0, 0), FULLSCREEN |
            # SCALED) raises "Cannot set 0 sized SCALED display mode".
            # Query the desktop size and pass it explicitly. SDL's
            # display info is reliable on Win32 + Pi OS.
            try:
                info = pygame.display.Info()
                desktop_w = info.current_w or 1920
                desktop_h = info.current_h or 1080
            except Exception:
                desktop_w, desktop_h = 1920, 1080
            flags = pygame.FULLSCREEN | pygame.SCALED
            screen = pygame.display.set_mode(
                (desktop_w, desktop_h), flags
            )
        else:
            flags = pygame.RESIZABLE
            screen = pygame.display.set_mode(
                (self.width, self.height), flags
            )
        pygame.display.set_caption("Glasshouse v2 — Demo")
        clock = pygame.time.Clock()

        # Fonts scale to screen height
        sw, sh = screen.get_size()
        font_headline = pygame.font.SysFont(None, max(int(sh * 0.10), 36))
        font_quad = pygame.font.SysFont(None, max(int(sh * 0.06), 28))
        font_meta = pygame.font.SysFont(None, max(int(sh * 0.035), 18))
        font_small = pygame.font.SysFont(None, max(int(sh * 0.025), 14))

        # Layout: top 70% = room view + headline, bottom 30% = link bars + status
        while not self._stop.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.stop_event.set()
                    self._stop.set()
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        self.stop_event.set()
                        self._stop.set()
                    elif event.key == pygame.K_r:
                        self.reset_event.set()
                        self.trigger_reset_visual()
                elif event.type == pygame.VIDEORESIZE:
                    sw, sh = event.size
                    screen = pygame.display.set_mode(
                        (sw, sh), pygame.RESIZABLE
                    )
                    font_headline = pygame.font.SysFont(
                        None, max(int(sh * 0.10), 36)
                    )
                    font_quad = pygame.font.SysFont(
                        None, max(int(sh * 0.06), 28)
                    )
                    font_meta = pygame.font.SysFont(
                        None, max(int(sh * 0.035), 18)
                    )
                    font_small = pygame.font.SysFont(
                        None, max(int(sh * 0.025), 14)
                    )

            with self._lock:
                state = dict(self._latest)

            screen.fill(_BG)
            sw, sh = screen.get_size()

            # Top 70%: room map + headline
            map_h = int(sh * 0.70)
            self._draw_room(screen, state, 0, 0, sw, map_h,
                              font_quad, font_headline, font_meta,
                              font_small)

            # Side panels mirror each other across the room map: the
            # room is portrait (3.81 x 9.14 m), so when it's centered
            # there are wide empty margins on both sides of the screen.
            # We use them: RX RATE on the right, LAST RESET on the
            # left. Each panel gets the full vertical budget between
            # the headline reserve and the link-bars row, so neither
            # has to clip its rows.
            panel_w = min(300, sw // 4)
            col_top = int(sh * 0.22)         # below headline + sub
            col_bottom = map_h - 12          # above link-bars row
            col_h = max(0, col_bottom - col_top)
            right_panel_x = sw - panel_w - 20
            left_panel_x = 20
            self._draw_rx_stats(screen, state,
                                 right_panel_x, col_top,
                                 panel_w, col_h,
                                 font_meta, font_small)
            self._draw_last_reset(screen, state,
                                   left_panel_x, col_top,
                                   panel_w, col_h,
                                   font_meta, font_small)

            # Bottom 30%: link bars + status
            self._draw_link_bars(screen, state, 0, map_h, sw,
                                   sh - map_h, font_meta, font_small)

            # Reset flash (within 1.5s of last reset)
            if state["last_reset_t"]:
                age = time.time() - state["last_reset_t"]
                if age < 1.5:
                    self._draw_reset_flash(screen, sw, sh,
                                              age / 1.5,
                                              font_meta)

            # Starvation warning banner
            if state["starvation_warning_active"]:
                self._draw_warning_banner(screen, sw, sh, font_meta)

            # Reset / quit hint in corner
            hint = font_small.render(
                "R = reset HMM   Q/ESC = quit",
                True, _TEXT_MUTED,
            )
            screen.blit(hint, (sw - hint.get_width() - 10,
                                sh - hint.get_height() - 8))

            pygame.display.flip()
            clock.tick(20)

        try:
            pygame.quit()
        except Exception:
            pass

    # ---- drawing helpers ----

    def _draw_room(self, screen, state, x0, y0, w, h,
                     font_quad, font_headline, font_meta, font_small):
        # Room rectangle in pixels: take the available area, fit a
        # 3.81 x 9.14 m portrait rectangle inside it. North is up.
        # Reserve more vertical space at the top for the headline +
        # BPM line so they don't overlap with the room top edge.
        margin = 40
        avail_w = w - 2 * margin
        head_reserve = int(h * 0.30)  # bigger reserve for headline+bpm
        avail_h = h - 2 * margin - head_reserve

        # Room aspect = width / length (3.81 / 9.14 = 0.417). Fit
        # rectangle to available area.
        room_aspect = self.room_width_m / self.room_length_m
        if avail_w / max(1, avail_h) > room_aspect:
            # available area wider than room ratio -> bound by height
            room_h = avail_h
            room_w = int(room_h * room_aspect)
        else:
            room_w = avail_w
            room_h = int(room_w / room_aspect)
        room_x = x0 + (w - room_w) // 2
        room_y = y0 + head_reserve + (avail_h - room_h) // 2

        # Headline at top of map area
        smoothed = state["smoothed_class"]
        conf = state["confidence"] or 0.0
        if smoothed:
            head_text = f"{smoothed}  ({int(conf * 100)}% confidence)"
            head_color = _LIT
        else:
            head_text = f"ANALYZING  ({int(conf * 100)}%)"
            head_color = _TEXT_MUTED
        surf = font_headline.render(head_text, True, head_color)
        screen.blit(surf, (x0 + (w - surf.get_width()) // 2,
                              y0 + int(h * 0.05)))

        # Sub-line: BPM
        bpm = state["bpm"]
        bpm_text = (f"breathing peak: {bpm:.0f} BPM"
                    if bpm else "no breathing peak detected")
        s2 = font_meta.render(bpm_text, True, _TEXT_MUTED)
        screen.blit(s2, (x0 + (w - s2.get_width()) // 2,
                            y0 + int(h * 0.16)))

        # Draw 4-quadrant room
        cell_w = room_w // 2
        cell_h = room_h // 2
        for q, (col, row) in _QUAD_CELL.items():
            cx = room_x + col * cell_w
            cy = room_y + row * cell_h
            color = _DARK
            if smoothed == q:
                # Pulse the lit quadrant for visual presence
                pulse = (math.sin(time.time() * 3.0) + 1.0) * 0.5
                lit = tuple(int(_LIT_DIM[i]
                                 + pulse * (_LIT[i] - _LIT_DIM[i]))
                             for i in range(3))
                color = lit
            pygame.draw.rect(screen, color, (cx, cy, cell_w, cell_h))
            pygame.draw.rect(screen, _BORDER, (cx, cy, cell_w, cell_h),
                              3)
            label = font_quad.render(q, True, _TEXT)
            screen.blit(label, (cx + 12, cy + 12))

        # Body asterisk + confidence circle
        pos = state["position_xy"]
        if pos and smoothed and self.show_body_marker:
            px_m = pos[0]
            py_m = pos[1]
            # Map (x in [0, room_width], y in [0, room_length]) to
            # pixel coords. y=0 is south (bottom), y=room_length is
            # north (top). x=0 is west (left), x=room_width is east
            # (right).
            px = room_x + int(px_m / self.room_width_m * room_w)
            py = room_y + int(
                (1.0 - py_m / self.room_length_m) * room_h
            )
            # Confidence-radius circle (large = uncertain, small = sure)
            base_r = int(min(room_w, room_h) * 0.18)
            r = max(8, int(base_r * (1.0 - conf)))
            # Draw soft glow circle
            for i, alpha in [(r + 12, 30), (r + 6, 60), (r, 120)]:
                surf = pygame.Surface((i * 2, i * 2),
                                        pygame.SRCALPHA)
                pygame.draw.circle(
                    surf, (*_CIRCLE, alpha), (i, i), i
                )
                screen.blit(surf, (px - i, py - i))
            # Asterisk dot
            pygame.draw.circle(screen, _ASTERISK, (px, py), 6)

    def _draw_rx_stats(self, screen, state, x0, y0, w, h,
                         font_meta, font_small):
        """Floating RX-rate panel.

        Big total IQ-record count over the last short window, plus
        per-node mini-bars. Each node's bar color matches its
        quadrant so the operator can read both rate health and
        spatial coverage in one glance.
        """
        n_total = state.get("n_records_short")
        per_node = state.get("per_node_record_counts") or {}
        window_s = state.get("window_seconds")
        if n_total is None and not per_node:
            return  # warming up: no data yet

        # Card background (semi-transparent rounded panel)
        card = pygame.Surface((w, h), pygame.SRCALPHA)
        card.fill((22, 22, 30, 210))
        pygame.draw.rect(card, _BORDER, (0, 0, w, h), 2,
                          border_radius=12)
        screen.blit(card, (x0, y0))

        pad = 16
        yy = y0 + pad

        # Header line
        hdr = font_meta.render("RX RATE", True, _TEXT)
        screen.blit(hdr, (x0 + pad, yy))
        yy += hdr.get_height() + 1

        if window_s is not None:
            sub = font_small.render(f"last {window_s:.1f}s",
                                     True, _TEXT_MUTED)
            screen.blit(sub, (x0 + pad, yy))
            yy += sub.get_height() + 6

        # Big total number
        if n_total is not None:
            big_size = max(int(h * 0.20), 44)
            big_font = pygame.font.SysFont(None, big_size, bold=True)
            big_color = _LIT if n_total > 0 else _TEXT_MUTED
            total_surf = big_font.render(f"{n_total:,}",
                                            True, big_color)
            screen.blit(total_surf, (x0 + pad, yy))
            yy += total_surf.get_height() - 6
            rec_lbl = font_small.render("IQ records", True,
                                          _TEXT_MUTED)
            screen.blit(rec_lbl, (x0 + pad, yy))
            yy += rec_lbl.get_height() + 10

        # Divider
        pygame.draw.line(screen, _BORDER,
                          (x0 + pad, yy),
                          (x0 + w - pad, yy), 1)
        yy += 10

        # Per-node mini-bars. Color each bar by the node's quadrant so
        # the operator gets a spatial read at the same time as a rate
        # read. Node 1=Q3, 2=Q1, 3=Q2, 4=Q4 — palette below.
        per_node_colors = {
            1: (170, 110, 220),   # Q3: violet
            2: (90, 170, 220),    # Q1: blue
            3: (240, 180, 90),    # Q2: amber
            4: (220, 110, 110),   # Q4: red-pink
        }
        per_node_label = {
            1: "N1 / Q3",
            2: "N2 / Q1",
            3: "N3 / Q2",
            4: "N4 / Q4",
        }

        nmax = max(per_node.values()) if per_node else 0
        # Reserve space: label (left) ~ 70px, count (right) ~ 40px,
        # bar in between.
        label_w = 72
        count_w = 44
        bar_x = x0 + pad + label_w
        bar_max_w = w - pad * 2 - label_w - count_w

        for nid in (1, 2, 3, 4):
            if yy + 22 > y0 + h - pad:
                break  # don't overflow card
            cnt = per_node.get(nid, 0)
            # Label
            lbl = font_small.render(per_node_label[nid], True, _TEXT)
            screen.blit(lbl, (x0 + pad, yy + 2))
            # Bar background
            b_h = 10
            b_y = yy + 5
            pygame.draw.rect(screen, _BAR_BG,
                              (bar_x, b_y, bar_max_w, b_h),
                              border_radius=3)
            # Bar fill (proportional to max across nodes — relative,
            # not absolute, because absolute capacity changes with
            # window length and per-node packet rate)
            if nmax > 0 and cnt > 0:
                fw = max(2, int(bar_max_w * cnt / nmax))
                pygame.draw.rect(screen, per_node_colors[nid],
                                  (bar_x, b_y, fw, b_h),
                                  border_radius=3)
            # Count (right-aligned within the row)
            cnt_surf = font_small.render(f"{cnt}", True, _TEXT)
            screen.blit(cnt_surf,
                         (x0 + w - pad - cnt_surf.get_width(),
                          yy + 2))
            yy += lbl.get_height() + 8

    def _draw_last_reset(self, screen, state, x0, y0, w, h,
                           font_meta, font_small):
        """Most-recent-reboot panel.

        Pulls last_reset_info (populated when the detector sees a
        telemetry frame whose uptime regressed) and renders a compact
        forensic card: which node rebooted, ESP-IDF reason label,
        wall-clock age, total reset count this run, and per-node
        crash_count from NVS.
        """
        info = state.get("last_reset_info")
        per_node_crash = state.get("per_node_crash_count") or {}
        n_total = state.get("n_total_resets") or 0

        # Card background
        card = pygame.Surface((w, h), pygame.SRCALPHA)
        # Card tint: amber when there's a recent reset, dark when none
        recent = (info is not None
                  and (time.time() - info.get("t_wall", 0.0)) < 60.0)
        if recent:
            card.fill((60, 30, 20, 220))
            border = (220, 130, 60)
        else:
            card.fill((22, 22, 30, 210))
            border = _BORDER
        pygame.draw.rect(card, border, (0, 0, w, h), 2,
                          border_radius=12)
        screen.blit(card, (x0, y0))

        pad = 16
        yy = y0 + pad

        # Header
        hdr_color = (255, 180, 80) if recent else _TEXT
        hdr = font_meta.render("LAST RESET", True, hdr_color)
        screen.blit(hdr, (x0 + pad, yy))
        # Total reset count, right-aligned in header row
        if n_total > 0:
            tot = font_small.render(f"{n_total} this run",
                                     True, _TEXT_MUTED)
            screen.blit(tot,
                         (x0 + w - pad - tot.get_width(),
                          yy + 3))
        yy += hdr.get_height() + 6

        if info is None:
            none_lbl = font_small.render("No resets seen",
                                            True, _TEXT_MUTED)
            screen.blit(none_lbl, (x0 + pad, yy))
            return

        nid = info.get("node_id")
        reason = info.get("reason_label") or "Unknown"
        ccount = info.get("crash_count")
        t_wall = info.get("t_wall", 0.0)
        age_s = max(0.0, time.time() - t_wall)

        # Big line: "N4 · Brownout"
        big_size = max(int(h * 0.22), 26)
        big_font = pygame.font.SysFont(None, big_size, bold=True)
        big_color = (255, 200, 90) if recent else _TEXT
        big_text = f"N{nid}  {reason}" if nid else reason
        big_surf = big_font.render(big_text, True, big_color)
        # Truncate if too wide
        if big_surf.get_width() > w - pad * 2:
            # Shrink to fit
            shrink_size = big_size
            while (big_surf.get_width() > w - pad * 2
                   and shrink_size > 14):
                shrink_size -= 2
                big_font = pygame.font.SysFont(None, shrink_size,
                                                  bold=True)
                big_surf = big_font.render(big_text, True, big_color)
        screen.blit(big_surf, (x0 + pad, yy))
        yy += big_surf.get_height() + 4

        # Sub: "12s ago  ·  crash_count=3"
        if age_s < 60:
            age_str = f"{int(age_s)}s ago"
        elif age_s < 3600:
            age_str = f"{int(age_s / 60)}m ago"
        else:
            age_str = f"{int(age_s / 3600)}h ago"
        ccount_str = (f"  ·  crash_count={ccount}"
                      if ccount is not None else "")
        sub = font_small.render(f"{age_str}{ccount_str}",
                                  True, _TEXT_MUTED)
        screen.blit(sub, (x0 + pad, yy))
        yy += sub.get_height() + 6

        # Per-node crash_count one-line summary, only if any > 0.
        crashes = [(n, c) for n, c in sorted(per_node_crash.items())
                   if c and c > 0]
        if crashes and (yy + 14) <= y0 + h - pad:
            crashes_str = "  ".join(f"N{n}:{c}" for n, c in crashes)
            cc = font_small.render(f"NVS crashes  {crashes_str}",
                                     True, _TEXT_MUTED)
            screen.blit(cc, (x0 + pad, yy))
            yy += cc.get_height() + 12

        # Recent reboot history. Renders as a list of one-line entries
        # under a small divider, oldest at top -> newest at bottom.
        # Skips the most-recent entry (already shown above as the
        # headline line) so the user doesn't read the same event twice.
        recent = state.get("recent_resets") or []
        if len(recent) > 1 and (yy + 30) <= y0 + h - pad:
            pygame.draw.line(screen, _BORDER,
                              (x0 + pad, yy),
                              (x0 + w - pad, yy), 1)
            yy += 8
            hdr2 = font_small.render("Recent reboots",
                                       True, _TEXT_MUTED)
            screen.blit(hdr2, (x0 + pad, yy))
            yy += hdr2.get_height() + 6
            now_t = time.time()
            # Show all but the most-recent (which is the headline),
            # newest-first (drop the last/headline entry, then reverse).
            history = list(recent[:-1])
            history.reverse()
            for item in history:
                if yy + 18 > y0 + h - pad:
                    break  # don't overflow
                _nid = item.get("node_id")
                _reason = item.get("reason_label") or "Unknown"
                _t = item.get("t_wall", 0.0)
                _age = max(0.0, now_t - _t)
                if _age < 60:
                    _age_str = f"{int(_age)}s"
                elif _age < 3600:
                    _age_str = f"{int(_age / 60)}m"
                else:
                    _age_str = f"{int(_age / 3600)}h"
                # Left side: Nx · Reason   Right side: 12s
                left_str = f"N{_nid}  {_reason}"
                left_surf = font_small.render(left_str, True, _TEXT)
                right_surf = font_small.render(_age_str,
                                                  True, _TEXT_MUTED)
                # Truncate left if it would overlap right
                max_left_w = w - pad * 2 - right_surf.get_width() - 12
                if left_surf.get_width() > max_left_w:
                    # crude truncation: shrink reason label to fit
                    while (left_surf.get_width() > max_left_w
                           and len(left_str) > 4):
                        left_str = left_str[:-1]
                        left_surf = font_small.render(left_str + "…",
                                                          True, _TEXT)
                screen.blit(left_surf, (x0 + pad, yy))
                screen.blit(right_surf,
                             (x0 + w - pad - right_surf.get_width(),
                              yy))
                yy += left_surf.get_height() + 4

    def _draw_link_bars(self, screen, state, x0, y0, w, h,
                         font_meta, font_small):
        # Title
        title = font_meta.render("Per-link breathing activity",
                                    True, _TEXT)
        screen.blit(title, (x0 + 20, y0 + 8))

        af = state["per_link_active_fraction"] or {}
        pkts = state["per_link_record_counts"] or {}
        streak = state["per_link_starve_streak"] or {}

        bar_area_x = x0 + 20
        bar_area_y = y0 + int(h * 0.30)
        bar_area_w = w - 40
        bar_area_h = int(h * 0.55)

        n = len(_LINKS_ORDERED)
        bar_w = (bar_area_w - (n - 1) * 16) // n
        for i, link in enumerate(_LINKS_ORDERED):
            bx = bar_area_x + i * (bar_w + 16)
            by = bar_area_y
            af_v = (af.get(link) or 0.0)
            streak_v = (streak.get(link) or 0)
            n_pkts = pkts.get(link, 0)
            # Background bar
            pygame.draw.rect(screen, _BAR_BG,
                              (bx, by, bar_w, bar_area_h))
            # Foreground (active_fraction filled bar)
            fill_h = int(bar_area_h * min(1.0, af_v))
            color = _BAR_STARVED if streak_v > 30 else _BAR_FG
            if fill_h > 0:
                pygame.draw.rect(
                    screen, color,
                    (bx, by + (bar_area_h - fill_h), bar_w, fill_h)
                )
            # Link label
            lbl = font_small.render(f"L{link.replace('-', '')}",
                                       True, _TEXT)
            screen.blit(lbl, (bx + (bar_w - lbl.get_width()) // 2,
                                 by + bar_area_h + 4))
            # Active fraction %
            af_pct = font_small.render(f"{af_v*100:.0f}%",
                                          True, _TEXT_MUTED)
            screen.blit(af_pct,
                          (bx + (bar_w - af_pct.get_width()) // 2,
                           by - af_pct.get_height() - 2))
            # Starvation streak indicator
            if streak_v > 0:
                streak_color = (_BANNER_WARN
                                if streak_v > 30 else _TEXT_MUTED)
                s_lbl = font_small.render(
                    f"sv {streak_v}", True, streak_color
                )
                screen.blit(s_lbl,
                              (bx + (bar_w - s_lbl.get_width()) // 2,
                               by + bar_area_h + 4
                               + lbl.get_height() + 2))

    def _draw_warning_banner(self, screen, sw, sh, font_meta):
        msg = ">>> RESTART COORD recommended (sustained link starvation)"
        s = font_meta.render(msg, True, (255, 255, 255))
        bw = s.get_width() + 30
        bh = s.get_height() + 10
        bx = (sw - bw) // 2
        by = 10
        # Pulse
        pulse = (math.sin(time.time() * 4.0) + 1.0) * 0.5
        color = tuple(int(_BANNER_WARN[i] * (0.5 + 0.5 * pulse))
                      for i in range(3))
        pygame.draw.rect(screen, color, (bx, by, bw, bh))
        screen.blit(s, (bx + 15, by + 5))

    def _draw_reset_flash(self, screen, sw, sh, t, font_meta):
        msg = "RESET — HMM cleared"
        s = font_meta.render(msg, True, (255, 255, 255))
        bw = s.get_width() + 30
        bh = s.get_height() + 12
        bx = (sw - bw) // 2
        by = sh - bh - 60
        alpha = int(255 * (1.0 - t))
        surf = pygame.Surface((bw, bh), pygame.SRCALPHA)
        surf.fill((*_BANNER_RESET, alpha))
        screen.blit(surf, (bx, by))
        screen.blit(s, (bx + 15, by + 6))


__all__ = ["PygameDisplay"]
