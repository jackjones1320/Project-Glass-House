"""Single source of truth for wire-format frame decoding.

Decodes the live UDP frame magics emitted by the perimeter firmware
(``firmware/perimeter/main/edge_processing.h``, ``csi_collector.h``,
``telemetry_sender.h``), the 10-byte link_reporter frame, and the
single-byte 0xAA heartbeat from ``heartbeat.c``.

Wire byte order: little-endian (ESP32-S3 native).
"""

from __future__ import annotations

import struct
from typing import Any


# --- Magic numbers (u32 LE on wire = low byte first in hex) ---
# 0xC5110001 -> b'\x01\x00\x11\xc5'
CSI_MAGIC_LE        = b'\x01\x00\x11\xc5'  # raw CSI frame
VITALS_MAGIC_LE     = b'\x02\x00\x11\xc5'  # on-device vitals (32 B)
IQ_MAGIC_LE         = b'\x06\x00\x11\xc5'  # raw IQ samples
TELEMETRY_MAGIC_LE  = b'\x09\x00\x11\xc5'  # structured telemetry (48 B)

CSI_HEADER_SIZE     = 20
VITALS_PKT_SIZE     = 32
TELEMETRY_PKT_SIZE  = 48


def parse_packet(packet: bytes) -> dict[str, Any]:
    """Decode one UDP payload into a typed record.

    Returns a dict with ``type`` set to one of:
        ``'heartbeat'`` | ``'csi'`` | ``'vitals'`` | ``'iq'`` |
        ``'telemetry'`` | ``'link'`` | ``'vitals_short'`` | ``'unknown'``

    Dispatch order:
      1. 1-byte 0xAA heartbeat (perimeter heartbeat.c).
      2. 4-byte magic match.
      3. 10-byte link_reporter frame (``packet[0] == 0x01`` and
         ``len == 10``).
      4. fallback: ``'unknown'``.
    """
    rec: dict[str, Any] = {"raw": packet.hex()}
    n = len(packet)

    # 1) Perimeter heartbeat — heartbeat.c sends a single 0xAA byte via UDP.
    if n == 1 and packet[0] == 0xAA:
        rec.update({"type": "heartbeat", "len": 1})
        return rec

    # 2) Magic dispatch. Only meaningful for frames >= 4 bytes.
    if n >= 4:
        magic4 = packet[:4]

        if magic4 == CSI_MAGIC_LE and n >= CSI_HEADER_SIZE:
            # CSI frame header layout (see firmware csi_collector.c).
            node_id, n_ant, n_subcar, freq_mhz, seq, rssi, noise_floor = struct.unpack_from(
                '<BBHIIbb', packet, 4
            )
            iq_actual = n - CSI_HEADER_SIZE
            iq_declared = n_subcar * 2 * max(n_ant, 1)
            rec.update({
                "type": "csi",
                "node_id": node_id,
                "n_antennas": n_ant,
                "n_subcarriers": n_subcar,
                "freq_mhz": freq_mhz,
                "seq": seq,
                "rssi": rssi,
                "noise_floor": noise_floor,
                "iq_bytes": iq_actual,
                "iq_bytes_declared": iq_declared,
                "len": n,
            })
            return rec

        if magic4 == VITALS_MAGIC_LE:
            # edge_vitals_pkt_t — edge_processing.h (32 bytes).
            if n >= VITALS_PKT_SIZE:
                flags = packet[5]
                energy = struct.unpack_from('<f', packet, 16)[0]
                rec.update({
                    "type": "vitals",
                    "node_id": packet[4],
                    "flags": flags,
                    "presence": bool(flags & 0x01),
                    "fall_bit": bool(flags & 0x02),
                    "motion_bit": bool(flags & 0x04),
                    "motion_energy": round(energy, 6),
                    "len": n,
                })
            else:
                rec.update({"type": "vitals_short", "len": n})
            return rec

        if magic4 == IQ_MAGIC_LE and n >= 8:
            # Two wire formats supported in parallel:
            #   v1 (8-byte header):  magic(4) node_id(1) channel(1)
            #                        iq_len(2) payload
            #   v2 (10-byte header): magic(4) node_id(1) peer_id(1) channel(1)
            #                        pad(1) iq_len(2) payload
            # Auto-detect by checking which iq_len position matches the packet
            # length (header_size + iq_len == n). Prefer v2 if both match.
            node_id = packet[4]
            v2_iq_len = struct.unpack_from('<H', packet, 8)[0] if n >= 10 else 0
            v1_iq_len = struct.unpack_from('<H', packet, 6)[0]
            v2_match = (n >= 10) and (10 + v2_iq_len == n)
            v1_match = (8 + v1_iq_len == n)
            if v2_match:
                rec.update({
                    "type": "iq",
                    "node_id": node_id,
                    "peer_id": packet[5],
                    "channel": packet[6],
                    "iq_len": v2_iq_len,
                    "wire_version": 2,
                    "len": n,
                })
            elif v1_match:
                rec.update({
                    "type": "iq",
                    "node_id": node_id,
                    "channel": packet[5],
                    "iq_len": v1_iq_len,
                    "wire_version": 1,
                    "len": n,
                })
            else:
                # Neither layout matches the actual packet length — corrupt
                # packet or unknown format. Fall back to v1 best-effort.
                rec.update({
                    "type": "iq",
                    "node_id": node_id,
                    "channel": packet[5],
                    "iq_len": v1_iq_len,
                    "wire_version": 0,
                    "len": n,
                })
            return rec

        if magic4 == TELEMETRY_MAGIC_LE and n >= TELEMETRY_PKT_SIZE:
            # Telemetry frame — 48 bytes, struct '<IBBHIBBHIIIIIIII'.
            # See firmware/perimeter/main/telemetry_sender.c for the producer.
            # The third field (uint16) is crash_count, the NVS-persisted
            # count of crashes since the last ESP_RST_POWERON.
            (_magic, node_id, reset_reason, crash_count,
             uptime_s,
             wifi_connected, _pad1, free_heap_kb,
             s_cb_count, s_send_ok, s_send_fail, s_rate_skip,
             s_enomem_total_events, s_enomem_suppressed, s_netdown_fail,
             rssi_pack) = struct.unpack('<IBBHIBBHIIIIIIII',
                                        packet[:TELEMETRY_PKT_SIZE])
            # rssi_pack byte layout: [current, min, max, reserved] each int8.
            rssi_current = struct.unpack_from('<b', bytes([rssi_pack & 0xff]))[0]
            rssi_min     = struct.unpack_from('<b', bytes([(rssi_pack >> 8) & 0xff]))[0]
            rssi_max     = struct.unpack_from('<b', bytes([(rssi_pack >> 16) & 0xff]))[0]
            rec.update({
                "type": "telemetry",
                "node_id": node_id,
                "reset_reason": reset_reason,
                "crash_count": crash_count,
                "uptime_s": uptime_s,
                "wifi_connected": bool(wifi_connected),
                "free_heap_kb": free_heap_kb,
                "s_cb_count": s_cb_count,
                "s_send_ok": s_send_ok,
                "s_send_fail": s_send_fail,
                "s_rate_skip": s_rate_skip,
                "s_enomem_total_events": s_enomem_total_events,
                "s_enomem_suppressed": s_enomem_suppressed,
                "s_netdown_fail": s_netdown_fail,
                "rssi_current": rssi_current,
                "rssi_min": rssi_min,
                "rssi_max": rssi_max,
                "len": n,
            })
            return rec

    # 3) link_reporter frame: exactly 10 bytes, packet[0] == 0x01, layout
    # '<BBBfBH'. See firmware/perimeter/main/link_reporter.c. NOT a
    # magic-family frame — distinguishable by length.
    if n == 10 and packet[0] == 0x01:
        try:
            _, node, partner, variance, state, count = struct.unpack('<BBBfBH', packet[:10])
            lo, hi = min(node, partner), max(node, partner)
            rec.update({
                "type": "link",
                "link": f"{lo}{hi}",
                "node": node,
                "partner": partner,
                "variance": round(variance, 6),
                "state": int(state),
                "count": count,
                "len": n,
            })
            return rec
        except struct.error:
            pass  # fall through to unknown

    # 4) Fallback.
    rec.update({"type": "unknown", "len": n})
    return rec
