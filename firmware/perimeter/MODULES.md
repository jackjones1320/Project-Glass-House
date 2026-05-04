# Perimeter Firmware - Module Inventory

The perimeter firmware produces the UDP wire stream the host detector
parses: per-frame CSI (0xC5110001), raw I/Q (0xC5110006), 1-byte
heartbeat (0xAA), 10-byte link-variance frames, and 1 Hz structured
telemetry (0xC5110009). Every module in `main/` participates in that
pipeline.

## Modules

| File | Role |
|---|---|
| `main/main.c` | Entry point: NVS init, WiFi STA, task launches. |
| `main/csi_collector.{c,h}` | Promiscuous CSI capture, peer-MAC filtering, periodic NDP probe driver. Emits per-frame CSI packets (magic 0xC5110001). |
| `main/edge_processing.{c,h}` | Per-link ring buffer, DSP pipeline, raw I/Q packet serialization (magic 0xC5110006). |
| `main/stream_sender.{c,h}` | UDP socket, sendto under mutex, ok/fail/skip counters. |
| `main/heartbeat.{c,h}` | 1-byte `0xAA` UDP liveness packet. |
| `main/link_reporter.{c,h}` | Packed 10-byte per-link variance frame. |
| `main/telemetry_sender.{c,h}` | 1 Hz structured telemetry frame (magic 0xC5110009). |
| `main/crash_counter.{c,h}` | NVS-backed persistent crash counter, surfaced via telemetry. |
| `main/send_watchdog.{c,h}` | Stream-stuck-state recovery (soft reconnect + hard reboot escalation). |
| `main/nvs_config.{c,h}` | Load WiFi creds, target IP/port, node_id, and peer MAC whitelist from NVS. |
| `main/power_mgmt.{c,h}` | `WIFI_PS_NONE` for full CSI rate. |
| `partitions_4mb.csv` | Flash partition layout (4 MB). |

## Coordinator firmware

The coordinator (`firmware/coordinator/`) is the SoftAP-host node that
perimeters associate to and that forwards UDP frames to the host:

| File | Role |
|---|---|
| `coordinator/main/main.c` | SoftAP setup, optional STA-to-operator-hotspot bridge, UDP forward loop. |
| `coordinator/sdkconfig.defaults` | TX buffer headroom + lwIP mailbox sizing. |
