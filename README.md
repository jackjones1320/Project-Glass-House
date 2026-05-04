# Glasshouse - CSI-based room-scale localizer

Glasshouse is a Wi-Fi Channel State Information (CSI) localizer for a
single body inside a four-corner room. Four ESP32-S3 perimeter nodes
capture CSI from each other's traffic, a fifth ESP32-S3 acts as the
SoftAP coordinator and forwards every UDP packet to a host laptop, and
a Python detector on the laptop turns the per-link CSI streams into a
continuous (x, y) position estimate plus a four-state quadrant
prediction smoothed by a forward-filter HMM.

The pipeline is deterministic. There is no model-training step, no
per-room calibration capture, and no machine-learning weights to
deploy. The detector reads raw I/Q frames, runs CARM-style SVD drift
cancellation, computes a Welch periodogram in the breathing band
(0.15-0.40 Hz), and falls through soft endpoint attribution and an
HMM smoother to produce the prediction. Everything that depends on
a body being in the room is physics-driven.

## Quick start

### Prerequisites

- Python 3.10 or newer on the host laptop
- ESP-IDF v5.x configured on the firmware build machine
- Four ESP32-S3 perimeter nodes plus one ESP32-S3 coordinator
- A 2.4 GHz Wi-Fi network (e.g. a Windows mobile hotspot) the
  coordinator's STA can join

### Install Python dependencies

From the repository root:

    pip install -r requirements.txt

`requirements.txt` pins `numpy>=1.24` and `pygame-ce>=2.5`. The detector
has no other runtime dependency.

### Flash the firmware

From `firmware/coordinator/`:

    idf.py set-target esp32s3
    idf.py build flash

The coordinator reads its operator-Wi-Fi credentials from NVS (or from
the Kconfig fallbacks under `idf.py menuconfig`) and joins the
operator hotspot. It bridges everything received on UDP 4210 to the
host's UDP 4211 over the operator-Wi-Fi link.

From `firmware/perimeter/`:

    ./build_firmware.ps1

The PowerShell helper wraps `idf.py build`. After the build succeeds,
flash each of the four perimeter boards. Each perimeter is provisioned
with a unique node id (1-4), the SoftAP credentials, and the
coordinator's IP and port via `firmware/perimeter/provision.py`. See
that script for the exact NVS keys.

### Run the localizer on the host

From `python/`:

    python -u -m live.deterministic_detector udp `
        --continuous-estimator --svd-drift-modes 1 --single-body `
        --localize --ascii-map --hmm-adjacency-weight 4.0 `
        --position-max-step-m 1.5 --pygame --pygame-fullscreen

The detector binds UDP 4211 by default, prints a per-second prediction
line to stdout, and (with `--pygame`) opens a fullscreen visualizer
that renders the four-quadrant room map, the body marker, the
per-link breathing-band activity bars, and the most-recent reboot
panel.

## Flag reference

- `--continuous-estimator` - soft endpoint attribution plus the HMM
  forward filter. The headline path; the flag remains for compatibility.
- `--svd-drift-modes 1` - drop the dominant temporal SVD mode per link
  to remove slow drift before the breathing-band FFT.
- `--single-body` - suppress all but the highest-SNR breathing peak
  when frequency clustering produces fragments.
- `--localize` - compute a continuous (x, y) estimate from
  wall-pair perturbation ratios.
- `--ascii-map` - render a small ASCII room map after every emission.
- `--hmm-adjacency-weight 4.0` - kinematic constraint that makes
  wall-adjacent quadrant transitions four times more likely than the
  diagonal-opposite jump.
- `--position-max-step-m 1.5` - cap per-update position movement at
  1.5 m, matching a brisk walk at 1.5 m/s on a one-second window.
- `--pygame` / `--pygame-fullscreen` - open the visualizer; press R
  in the pygame window to reset the HMM and the position smoother;
  Q or ESC to quit.

See `python -m live.deterministic_detector udp --help` for the full
list, including the rolling-baseline, starvation-warning, and
blockage-attribution flags.

## Wire formats

The host parses four UDP frame families plus the coordinator
heartbeat. Magic numbers are little-endian on the wire (ESP32-S3
native).

| Magic | Source | Payload |
|---|---|---|
| `0xC5110001` | `csi_collector` | 20-byte header followed by raw I/Q bytes; one frame per CSI callback |
| `0xC5110006` | `edge_processing` | Per-link I/Q packet (8- or 10-byte header plus I/Q payload) used by the host detector |
| `0xC5110009` | `telemetry_sender` | 48-byte structured telemetry frame at 1 Hz (uptime, crash counter, RSSI window, send/skip counts) |
| (none) | `link_reporter` | 10-byte packed per-link variance frame: type, node id, partner id, variance, state, sample count |
| `0xAA` | `heartbeat` | Single-byte UDP keepalive emitted at 50 ms cadence |

The coordinator forwards every UDP datagram it receives on port 4210
verbatim to the host on port 4211. No COBS framing is applied on the
Wi-Fi forward path; UDP is already framed.

## Architecture

Each perimeter board runs the WiFi CSI callback in promiscuous mode,
filters frames to the registered peer-MAC whitelist, and pushes raw
I/Q into the edge-processing ring buffer. A 50 Hz NDP probe
stimulates the peers; their replies stream back into our CSI
callback, producing per-link CSI at roughly 50 Hz when the network
is healthy. Each frame is serialized into a `0xC5110006` packet and
sent to the coordinator over UDP. The 1 Hz telemetry frame and the
10-byte link-reporter frame ride the same socket.

The coordinator runs an open SoftAP on channel 1 (HT40+) so all four
perimeters can associate without a WPA2 handshake bottleneck. Its
UDP bridge listens on port 4210 and forwards every datagram, raw,
to the operator host on port 4211.

The host runs `live.deterministic_detector` which decodes each frame
via `frame_decoder.parse_packet`, buffers I/Q records into a 6-second
short window (variance gate) and a 90-second long window (Doppler
breathing-band FFT), and on each emission tick:

1. Computes per-link mean amplitude with Hampel/MAD outlier rejection.
2. Removes the leading SVD temporal mode per link to cancel drift.
3. Runs Welch's method on the breathing band per link.
4. Soft-attributes each link's signal score to its two endpoint
   nodes, weighted by sqrt(n_packets / max_n_packets).
5. Updates the four-state HMM forward filter; the argmax of the
   posterior is the displayed quadrant.
6. Maps wall-pair perturbation ratios into a continuous (x, y)
   position estimate, optionally clamped by `--position-max-step-m`
   and EMA-smoothed.

The pygame view layer (`live.pygame_display`) reads the per-emission
state through a thread-safe accessor and renders the room map, the
body marker, the per-link bars, and the most-recent-reset diagnostic
panel at 20 Hz.
