#!/usr/bin/env python3
"""
Finds the ESP32's current IP address by its MAC address, then pings it
continuously at a given interval.

Why by MAC and not IP: the board's IP can change between boots (normal
DHCP behavior, confirmed in docs/v1.md sec 6.1) -- its MAC address does
not. The default below is this project's ESP32-S3's WiFi station MAC,
printed once at boot as "wifi:mode : sta (xx:xx:xx:xx:xx:xx)". If you're
running this against different hardware, pass --mac.

How it finds the IP (same method used interactively in this project's
history, see docs/v1.md): read this machine's ARP/neighbor table for that
MAC. If it's not there yet (this machine has never resolved that address
before -- normal, since it usually talks to the board over USB serial, not
WiFi), do a quick ping sweep of the local /24 first. A sweep works even
though most individual pings in it are irrelevant or fail: to ping any
address on the local network at all, the OS has to resolve it to a MAC via
ARP first, and that resolution is what populates the table -- the ping
"succeeding" isn't actually the point.

Usage:
    python3 find_and_ping.py
    python3 find_and_ping.py --mac d4:05:92:7b:15:a4 --interval 0.2
    python3 find_and_ping.py --subnet 192.168.1.0/24
"""

import argparse
import ipaddress
import re
import socket
import subprocess
import sys

DEFAULT_MAC = "d4:05:92:7b:15:a4"  # this project's ESP32-S3, WiFi station MAC


def local_subnet():
    """Best-effort guess at this machine's own /24, via the same no-traffic
    trick server.py's local_lan_ip() uses: open a UDP socket toward a
    public address (nothing is actually sent for a UDP connect()) just to
    ask the OS which local interface/IP it would use."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    finally:
        s.close()
    return ipaddress.ip_network(f"{local_ip}/24", strict=False)


def ping_sweep(network):
    """Fire one quick ping at every host address in the subnet, in
    parallel, and wait for them all. Individual results are ignored --
    see the module docstring for why this still works."""
    print(f"Sweeping {network} ({network.num_addresses - 2} addresses)...")
    procs = [
        subprocess.Popen(["ping", "-c", "1", "-W", "1", str(ip)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for ip in network.hosts()
    ]
    for p in procs:
        p.wait()


def find_ip_by_mac(mac):
    """Read this machine's ARP/neighbor table, return the IP paired with
    the given MAC, or None."""
    mac = mac.lower()
    out = None
    for cmd in (["ip", "neigh", "show"], ["arp", "-a"]):
        try:
            out = subprocess.check_output(cmd, text=True)
            break
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    if out is None:
        return None

    for line in out.splitlines():
        if mac in line.lower():
            m = re.search(r"\b(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\b", line)
            if m:
                return m.group(1)
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mac", default=DEFAULT_MAC,
                         help=f"target MAC address (default: {DEFAULT_MAC})")
    parser.add_argument("--interval", type=float, default=0.2,
                         help="seconds between pings once found (default: 0.2 -- "
                              "intervals below 0.2s typically require root/sudo)")
    parser.add_argument("--subnet", default=None,
                         help="override the /24 to sweep, e.g. 192.168.1.0/24 "
                              "(default: auto-detected from this machine's own IP)")
    args = parser.parse_args()

    network = ipaddress.ip_network(args.subnet) if args.subnet else local_subnet()

    print(f"Looking for MAC {args.mac} ...")
    ip = find_ip_by_mac(args.mac)

    if ip is None:
        print("Not in the ARP table yet.")
        ping_sweep(network)
        ip = find_ip_by_mac(args.mac)

    if ip is None:
        print(f"\nCould not find a device with MAC {args.mac} on {network}.", file=sys.stderr)
        print("Check the board is powered on, connected to WiFi, and on the "
              "same network as this machine (or pass --subnet).", file=sys.stderr)
        sys.exit(1)

    print(f"Found it: {ip}")
    print(f"Pinging every {args.interval}s. Ctrl+C to stop.\n")

    try:
        subprocess.run(["ping", "-i", str(args.interval), ip])
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        print("'ping' not found on this system.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
