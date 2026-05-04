#!/usr/bin/env python3
"""
ESP32-S3 CSI Node Provisioning Script

Writes WiFi credentials, aggregator target, node ID, and peer-MAC
whitelist to the ESP32's NVS partition so users can configure a
pre-built firmware binary without recompiling.

Usage:
    python provision.py --port COM7 --ssid "MyWiFi" --password "secret" \
        --target-ip 192.168.1.20 --node-id 1 \
        --peer1-id 2 --peer1-mac AA:BB:CC:DD:EE:01 \
        --peer2-id 3 --peer2-mac AA:BB:CC:DD:EE:02 \
        --peer3-id 4 --peer3-mac AA:BB:CC:DD:EE:03

Requirements:
    pip install esptool esp-idf-nvs-partition-gen
    (or use the nvs_partition_gen.py bundled with ESP-IDF)
"""

import argparse
import csv
import io
import os
import subprocess
import sys
import tempfile


# NVS partition table offset for the standard 4 MB ESP-IDF partition
# scheme: the "nvs" partition starts at 0x9000 (36864) and is 0x6000
# (24576) bytes. Verify against firmware/perimeter/partitions_4mb.csv
# if you change the partition layout.
NVS_PARTITION_OFFSET = 0x9000
NVS_PARTITION_SIZE = 0x6000  # 24 KiB


def build_nvs_csv(args):
    """Build an NVS CSV string for the csi_cfg namespace."""
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["key", "type", "encoding", "value"])
    writer.writerow(["csi_cfg", "namespace", "", ""])
    if args.ssid:
        writer.writerow(["ssid", "data", "string", args.ssid])
    if args.password is not None:
        writer.writerow(["password", "data", "string", args.password])
    if args.target_ip:
        writer.writerow(["target_ip", "data", "string", args.target_ip])
    if args.target_port is not None:
        writer.writerow(["target_port", "data", "u16", str(args.target_port)])
    if args.node_id is not None:
        writer.writerow(["node_id", "data", "u8", str(args.node_id)])
    # Peer MAC whitelist for pairwise CSI sensing
    peer_entries = []
    for i in range(1, 5):
        pid = getattr(args, f"peer{i}_id", None)
        pmac = getattr(args, f"peer{i}_mac", None)
        if pid is not None and pmac is not None:
            peer_entries.append((i, pid, pmac))
    if peer_entries:
        writer.writerow(["peer_count", "data", "u8", str(len(peer_entries))])
        for idx, pid, pmac in peer_entries:
            writer.writerow([f"peer{idx}_id", "data", "u8", str(pid)])
            mac_bytes = bytes(int(b, 16) for b in pmac.split(":"))
            writer.writerow([f"peer{idx}_mac", "data", "hex2bin", mac_bytes.hex()])
    return buf.getvalue()


def generate_nvs_binary(csv_content, size):
    """Generate an NVS partition binary from CSV using nvs_partition_gen.py."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f_csv:
        f_csv.write(csv_content)
        csv_path = f_csv.name

    bin_path = csv_path.replace(".csv", ".bin")

    try:
        # Method 1: subprocess invocation (most reliable across package versions)
        for module_name in ["esp_idf_nvs_partition_gen", "nvs_partition_gen"]:
            try:
                subprocess.check_call(
                    [sys.executable, "-m", module_name, "generate",
                     csv_path, bin_path, hex(size)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
                with open(bin_path, "rb") as f:
                    return f.read()
            except (subprocess.CalledProcessError, FileNotFoundError):
                continue

        # Method 2: ESP-IDF bundled script
        idf_path = os.environ.get("IDF_PATH", "")
        gen_script = os.path.join(idf_path, "components", "nvs_flash",
                                  "nvs_partition_generator", "nvs_partition_gen.py")
        if os.path.isfile(gen_script):
            subprocess.check_call([
                sys.executable, gen_script, "generate",
                csv_path, bin_path, hex(size)
            ])
            with open(bin_path, "rb") as f:
                return f.read()

        raise RuntimeError(
            "NVS partition generator not available. "
            "Install: pip install esp-idf-nvs-partition-gen"
        )

    finally:
        for p in (csv_path, bin_path):
            if os.path.isfile(p):
                os.unlink(p)


def flash_nvs(port, baud, nvs_bin):
    """Flash the NVS partition binary to the ESP32."""
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(nvs_bin)
        bin_path = f.name

    try:
        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32s3",
            "--port", port,
            "--baud", str(baud),
            "write_flash",
            hex(NVS_PARTITION_OFFSET), bin_path,
        ]
        print(f"Flashing NVS partition ({len(nvs_bin)} bytes) to {port}...")
        subprocess.check_call(cmd)
        print("NVS provisioning complete!")
    finally:
        os.unlink(bin_path)


def main():
    parser = argparse.ArgumentParser(
        description="Provision ESP32-S3 CSI Node with WiFi and aggregator settings",
        epilog="Example: python provision.py --port COM7 --ssid MyWiFi --password secret --target-ip 192.168.1.20",
    )
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM7, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=460800, help="Flash baud rate (default: 460800)")
    parser.add_argument("--ssid", help="WiFi SSID (the coordinator's SoftAP)")
    parser.add_argument("--password", help="WiFi password")
    parser.add_argument("--target-ip", help="Aggregator host IP (e.g. 192.168.1.20)")
    parser.add_argument("--target-port", type=int, help="Aggregator UDP port (default: 5005)")
    parser.add_argument("--node-id", type=int, help="Node ID 1-4")
    # Peer MAC whitelist
    for i in range(1, 5):
        parser.add_argument(f"--peer{i}-id", type=int,
                            help=f"Peer {i} node ID (1-4)")
        parser.add_argument(f"--peer{i}-mac", type=str,
                            help=f"Peer {i} MAC address (AA:BB:CC:DD:EE:FF)")
    parser.add_argument("--dry-run", action="store_true", help="Generate NVS binary but don't flash")

    args = parser.parse_args()

    has_value = any([
        args.ssid, args.password is not None, args.target_ip,
        args.target_port, args.node_id is not None,
        args.peer1_id is not None,
    ])
    if not has_value:
        parser.error("At least one config value must be specified")

    # Validate peer MAC arguments: id and mac must be paired
    for i in range(1, 5):
        pid = getattr(args, f"peer{i}_id", None)
        pmac = getattr(args, f"peer{i}_mac", None)
        if (pid is not None) != (pmac is not None):
            parser.error(f"--peer{i}-id and --peer{i}-mac must be specified together")
        if pmac is not None:
            parts = pmac.split(":")
            if len(parts) != 6:
                parser.error(f"--peer{i}-mac must be in AA:BB:CC:DD:EE:FF format, got '{pmac}'")
            try:
                for p in parts:
                    val = int(p, 16)
                    if val < 0 or val > 255:
                        raise ValueError
            except ValueError:
                parser.error(f"--peer{i}-mac contains invalid hex bytes: '{pmac}'")

    print("Building NVS configuration:")
    if args.ssid:
        print(f"  WiFi SSID:     {args.ssid}")
    if args.password is not None:
        print(f"  WiFi Password: {'*' * len(args.password)}")
    if args.target_ip:
        print(f"  Target IP:     {args.target_ip}")
    if args.target_port:
        print(f"  Target Port:   {args.target_port}")
    if args.node_id is not None:
        print(f"  Node ID:       {args.node_id}")
    for i in range(1, 5):
        pid = getattr(args, f"peer{i}_id", None)
        pmac = getattr(args, f"peer{i}_mac", None)
        if pid is not None and pmac is not None:
            print(f"  Peer {i}:        node_id={pid} mac={pmac}")

    csv_content = build_nvs_csv(args)

    try:
        nvs_bin = generate_nvs_binary(csv_content, NVS_PARTITION_SIZE)
    except Exception as e:
        print(f"\nError generating NVS binary: {e}", file=sys.stderr)
        print("\nFallback: save CSV and flash manually with ESP-IDF tools.", file=sys.stderr)
        fallback_path = "nvs_config.csv"
        with open(fallback_path, "w") as f:
            f.write(csv_content)
        print(f"Saved NVS CSV to {fallback_path}", file=sys.stderr)
        print(f"Flash with: python $IDF_PATH/components/nvs_flash/"
              f"nvs_partition_generator/nvs_partition_gen.py generate "
              f"{fallback_path} nvs.bin 0x6000", file=sys.stderr)
        sys.exit(1)

    if args.dry_run:
        out = "nvs_provision.bin"
        with open(out, "wb") as f:
            f.write(nvs_bin)
        print(f"NVS binary saved to {out} ({len(nvs_bin)} bytes)")
        print(f"Flash manually: python -m esptool --chip esp32s3 --port {args.port} "
              f"write_flash 0x9000 {out}")
        return

    flash_nvs(args.port, args.baud, nvs_bin)


if __name__ == "__main__":
    main()
