#!/usr/bin/env python3
"""
ESP32-S3 Mesh Network Provisioning Script

Writes mesh network configuration to the ESP32's NVS partition for both
sensor nodes and the gateway node. Supports the ESP-MESH backhaul firmware
variants (esp32-csi-mesh-node and esp32-mesh-gateway).

Usage:
    # Provision a sensor node
    python provision-mesh.py --port /dev/ttyUSB0 \\
      --mode sensor \\
      --mesh-id "RUVIEW" \\
      --mesh-password "fieldpassword" \\
      --mesh-channel 6 \\
      --node-id 3

    # Provision the gateway node
    python provision-mesh.py --port /dev/ttyUSB0 \\
      --mode gateway \\
      --mesh-id "RUVIEW" \\
      --mesh-password "fieldpassword" \\
      --mesh-channel 6 \\
      --router-ssid "RuView-Field" \\
      --router-password "routerpassword" \\
      --aggregator-ip 192.168.8.100 \\
      --aggregator-port 5005

Requirements:
    pip install esptool
    (optionally: nvs-partition-gen, or use ESP-IDF's bundled script)
"""

import argparse
import csv
import io
import os
import struct
import subprocess
import sys
import tempfile


# NVS partition offset — default for ESP-IDF 4MB flash
NVS_PARTITION_OFFSET = 0x9000
NVS_PARTITION_SIZE = 0x6000  # 24 KiB


def build_sensor_nvs_csv(mesh_id, mesh_password, mesh_channel, node_id,
                         router_ssid, router_password):
    """Build NVS CSV for a sensor node (namespace: mesh_node)."""
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["key", "type", "encoding", "value"])
    writer.writerow(["mesh_node", "namespace", "", ""])

    if node_id is not None:
        writer.writerow(["node_id", "data", "u8", str(node_id)])

    if mesh_id is not None:
        # Mesh ID is a 6-byte blob — pad/truncate to exactly 6 bytes
        mid = mesh_id.encode("utf-8")[:6].ljust(6, b"\x00")
        # Store as hex string for NVS blob
        hex_str = "".join(f"{b:02x}" for b in mid)
        writer.writerow(["mesh_id", "data", "hex2bin", hex_str])

    if mesh_password is not None:
        writer.writerow(["mesh_pass", "data", "string", mesh_password])

    if mesh_channel is not None:
        writer.writerow(["mesh_chan", "data", "u8", str(mesh_channel)])

    if router_ssid is not None:
        writer.writerow(["router_ssid", "data", "string", router_ssid])

    if router_password is not None:
        writer.writerow(["router_pass", "data", "string", router_password])

    return buf.getvalue()


def build_gateway_nvs_csv(mesh_id, mesh_password, mesh_channel,
                          router_ssid, router_password,
                          aggregator_ip, aggregator_port):
    """Build NVS CSV for the gateway node (namespace: mesh_gw)."""
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["key", "type", "encoding", "value"])
    writer.writerow(["mesh_gw", "namespace", "", ""])

    if mesh_id is not None:
        mid = mesh_id.encode("utf-8")[:6].ljust(6, b"\x00")
        hex_str = "".join(f"{b:02x}" for b in mid)
        writer.writerow(["mesh_id", "data", "hex2bin", hex_str])

    if mesh_password is not None:
        writer.writerow(["mesh_pass", "data", "string", mesh_password])

    if mesh_channel is not None:
        writer.writerow(["mesh_chan", "data", "u8", str(mesh_channel)])

    if router_ssid is not None:
        writer.writerow(["router_ssid", "data", "string", router_ssid])

    if router_password is not None:
        writer.writerow(["router_pass", "data", "string", router_password])

    if aggregator_ip is not None:
        writer.writerow(["agg_ip", "data", "string", aggregator_ip])

    if aggregator_port is not None:
        writer.writerow(["agg_port", "data", "u16", str(aggregator_port)])

    return buf.getvalue()


def generate_nvs_binary(csv_content, size):
    """Generate an NVS partition binary from CSV."""
    with tempfile.NamedTemporaryFile(
            mode="w", suffix=".csv", delete=False) as f_csv:
        f_csv.write(csv_content)
        csv_path = f_csv.name

    bin_path = csv_path.replace(".csv", ".bin")

    try:
        # Try pip-installed version
        try:
            import nvs_partition_gen
            nvs_partition_gen.generate(csv_path, bin_path, size)
            with open(bin_path, "rb") as f:
                return f.read()
        except ImportError:
            pass

        # Try ESP-IDF bundled script
        idf_path = os.environ.get("IDF_PATH", "")
        gen_script = os.path.join(
            idf_path, "components", "nvs_flash",
            "nvs_partition_generator", "nvs_partition_gen.py")
        if os.path.isfile(gen_script):
            subprocess.check_call([
                sys.executable, gen_script, "generate",
                csv_path, bin_path, hex(size)
            ])
            with open(bin_path, "rb") as f:
                return f.read()

        # Last resort: try as module
        subprocess.check_call([
            sys.executable, "-m", "nvs_partition_gen", "generate",
            csv_path, bin_path, hex(size)
        ])
        with open(bin_path, "rb") as f:
            return f.read()

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
        description="Provision ESP32-S3 mesh nodes with network configuration",
        epilog=(
            "Examples:\n"
            "  # Sensor node:\n"
            "  python provision-mesh.py --port /dev/ttyUSB0 --mode sensor "
            "--mesh-id RUVIEW --mesh-password secret --node-id 3\n\n"
            "  # Gateway node:\n"
            "  python provision-mesh.py --port /dev/ttyUSB0 --mode gateway "
            "--mesh-id RUVIEW --mesh-password secret "
            "--router-ssid MyRouter --router-password pass "
            "--aggregator-ip 192.168.8.100"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--port", required=True,
        help="Serial port (e.g., /dev/ttyUSB0, COM7)")
    parser.add_argument(
        "--baud", type=int, default=460800,
        help="Flash baud rate (default: 460800)")
    parser.add_argument(
        "--mode", required=True, choices=["sensor", "gateway"],
        help="Node type: 'sensor' for CSI mesh node, 'gateway' for root bridge")

    # Mesh network settings (shared)
    mesh_group = parser.add_argument_group("Mesh network")
    mesh_group.add_argument(
        "--mesh-id", default="RUVIEW",
        help="6-char mesh network ID (default: RUVIEW)")
    mesh_group.add_argument(
        "--mesh-password",
        help="Mesh AP password")
    mesh_group.add_argument(
        "--mesh-channel", type=int,
        help="WiFi channel for mesh (1-13, default: 6)")

    # Sensor node settings
    sensor_group = parser.add_argument_group("Sensor node")
    sensor_group.add_argument(
        "--node-id", type=int,
        help="Node ID 1-254 (required for sensor mode)")

    # Gateway/router settings
    gw_group = parser.add_argument_group("Gateway / Router")
    gw_group.add_argument(
        "--router-ssid",
        help="Travel router SSID (required for gateway mode)")
    gw_group.add_argument(
        "--router-password",
        help="Travel router password")
    gw_group.add_argument(
        "--aggregator-ip",
        help="Aggregator IP address (required for gateway mode)")
    gw_group.add_argument(
        "--aggregator-port", type=int, default=5005,
        help="Aggregator UDP port (default: 5005)")

    parser.add_argument(
        "--dry-run", action="store_true",
        help="Generate NVS binary but don't flash")

    args = parser.parse_args()

    # Validate mode-specific requirements
    if args.mode == "sensor" and args.node_id is None:
        parser.error("--node-id is required for sensor mode")
    if args.mode == "sensor" and args.node_id is not None:
        if args.node_id < 1 or args.node_id > 254:
            parser.error("--node-id must be 1-254")
    if args.mode == "gateway" and args.router_ssid is None:
        parser.error("--router-ssid is required for gateway mode")
    if args.mode == "gateway" and args.aggregator_ip is None:
        parser.error("--aggregator-ip is required for gateway mode")
    if args.mesh_channel is not None and (
            args.mesh_channel < 1 or args.mesh_channel > 13):
        parser.error("--mesh-channel must be 1-13")

    # Print configuration summary
    print(f"\nProvisioning {args.mode.upper()} node:")
    print(f"  Mesh ID:       {args.mesh_id}")
    if args.mesh_password:
        print(f"  Mesh Password: {'*' * len(args.mesh_password)}")
    if args.mesh_channel:
        print(f"  Mesh Channel:  {args.mesh_channel}")

    if args.mode == "sensor":
        print(f"  Node ID:       {args.node_id}")
        if args.router_ssid:
            print(f"  Router SSID:   {args.router_ssid}")
        csv_content = build_sensor_nvs_csv(
            mesh_id=args.mesh_id,
            mesh_password=args.mesh_password,
            mesh_channel=args.mesh_channel,
            node_id=args.node_id,
            router_ssid=args.router_ssid,
            router_password=args.router_password,
        )
    else:
        print(f"  Router SSID:   {args.router_ssid}")
        if args.router_password:
            print(f"  Router Pass:   {'*' * len(args.router_password)}")
        print(f"  Aggregator:    {args.aggregator_ip}:{args.aggregator_port}")
        csv_content = build_gateway_nvs_csv(
            mesh_id=args.mesh_id,
            mesh_password=args.mesh_password,
            mesh_channel=args.mesh_channel,
            router_ssid=args.router_ssid,
            router_password=args.router_password,
            aggregator_ip=args.aggregator_ip,
            aggregator_port=args.aggregator_port,
        )

    try:
        nvs_bin = generate_nvs_binary(csv_content, NVS_PARTITION_SIZE)
    except Exception as e:
        print(f"\nError generating NVS binary: {e}", file=sys.stderr)
        print("\nFallback: saving CSV for manual flash.", file=sys.stderr)
        fallback_path = f"nvs_mesh_{args.mode}.csv"
        with open(fallback_path, "w") as f:
            f.write(csv_content)
        print(f"Saved NVS CSV to {fallback_path}", file=sys.stderr)
        print(
            "Generate binary with:\n"
            f"  python $IDF_PATH/components/nvs_flash/"
            f"nvs_partition_generator/nvs_partition_gen.py generate "
            f"{fallback_path} nvs.bin 0x6000\n"
            "Then flash with:\n"
            f"  python -m esptool --chip esp32s3 --port {args.port} "
            f"write_flash 0x9000 nvs.bin",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.dry_run:
        out = f"nvs_mesh_{args.mode}.bin"
        with open(out, "wb") as f:
            f.write(nvs_bin)
        print(f"\nNVS binary saved to {out} ({len(nvs_bin)} bytes)")
        print(
            f"Flash manually:\n"
            f"  python -m esptool --chip esp32s3 --port {args.port} "
            f"write_flash 0x9000 {out}")
        return

    flash_nvs(args.port, args.baud, nvs_bin)


if __name__ == "__main__":
    main()
