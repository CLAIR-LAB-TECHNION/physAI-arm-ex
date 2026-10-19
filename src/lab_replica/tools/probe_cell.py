#!/usr/bin/env python3
"""Read-only probe of the real cell: UR model and generation, and how the RG6 is connected.

    python3 probe_cell.py <robot ip> [--compute-box IP ...] [--scan]

Standard library only, so it can be copied to any machine on the robot's
network. It never powers, loads, starts or moves anything:

* Dashboard server (29999): only status queries (model, serial, PolyScope
  version, robot/safety mode, loaded program, remote control).
* Primary interface (30011 read-only, else 30001): reads the broadcast state
  for a few seconds and sends nothing. The DH parameters identify the model
  even if the dashboard does not know get_robot_model; the tool data shows
  whether the tool flange powers something (an RG6 cabled to the flange).
* OnRobot Compute Box: Modbus TCP read of the RG status registers 258-275
  (unit 65) at 192.168.1.1, the given --compute-box addresses and, with
  --scan, every host of the robot's /24 that has port 502 open.
"""

import argparse
import ipaddress
import socket
import struct
import time
from concurrent.futures import ThreadPoolExecutor

DASHBOARD_QUERIES = [
    "get_robot_model", "get_serial_number", "PolyscopeVersion", "version",
    "robotmode", "safetystatus", "safetymode", "get_operational_mode",
    "is in remote control", "programState", "get_loaded_program",
]

# Nominal DH parameters (d1, a2, a3) from ur_description / UR's DH tables.
NOMINAL_DH = {
    "UR3 (CB3)": (0.1519, -0.24365, -0.21325),
    "UR5 (CB3)": (0.089159, -0.425, -0.39225),
    "UR10 (CB3)": (0.1273, -0.612, -0.5723),
    "UR3e": (0.15185, -0.24355, -0.2132),
    "UR5e": (0.1625, -0.425, -0.3922),
    "UR10e": (0.1807, -0.6127, -0.57155),
    "UR16e": (0.1807, -0.4784, -0.36),
    "UR20": (0.2363, -0.862, -0.7287),
    "UR30": (0.2363, -0.6370, -0.5037),
}

HOME_JOINTS = [0.1935, -1.9261, 1.6783, -1.3231, -1.5708, 0.1935]


def dashboard(ip):
    print(f"\n== Dashboard server {ip}:29999")
    try:
        s = socket.create_connection((ip, 29999), timeout=3)
    except OSError as e:
        print(f"  not reachable: {e}")
        return
    f = s.makefile("rwb")
    print(f"  banner: {f.readline().decode(errors='replace').strip()}")
    for q in DASHBOARD_QUERIES:
        f.write((q + "\n").encode())
        f.flush()
        print(f"  {q:22s} -> {f.readline().decode(errors='replace').strip()}")
    s.close()


def read_primary(ip, seconds=3.0):
    """Collect primary-interface packets; returns (port, list of (type, payload))."""
    for port in (30011, 30001):
        try:
            s = socket.create_connection((ip, port), timeout=3)
        except OSError:
            continue
        buf, packets, t_end = b"", [], time.time() + seconds
        while time.time() < t_end:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 5:
                (n,) = struct.unpack_from(">i", buf)
                if n < 5 or len(buf) < n:
                    break
                packets.append((buf[4], buf[5:n]))
                buf = buf[n:]
        s.close()
        return port, packets
    return None, []


def sub_packages(payload):
    i = 0
    while i + 5 <= len(payload):
        (n,) = struct.unpack_from(">i", payload, i)
        if n < 5:
            return
        yield payload[i + 4], payload[i + 5:i + n]
        i += n


def primary(ip):
    port, packets = read_primary(ip)
    print(f"\n== Primary interface {ip}:{port or '30011/30001'}")
    if port is None:
        print("  not reachable")
        return
    state = {}
    for ptype, p in packets:
        if ptype == 20 and len(p) >= 10 and p[9] == 3:          # version message
            k = p[10]
            name = p[11:11 + k].decode(errors="replace")
            major, minor = p[11 + k], p[12 + k]
            bugfix, build = struct.unpack_from(">ii", p, 13 + k)
            state["version"] = f"{name} {major}.{minor}.{bugfix}.{build}"
        elif ptype == 16:                                       # robot state
            for st, sp in sub_packages(p):
                state.setdefault(st, sp)
    print(f"  controller software: {state.get('version', 'no version message seen')}")

    if 5 in state and len(state[5]) >= 24 + 4 * 48:             # kinematics info
        dh = struct.unpack_from(">24d", state[5], 24)
        a, d = dh[6:12], dh[12:18]
        print(f"  DH a = {[round(x, 5) for x in a]}")
        print(f"  DH d = {[round(x, 5) for x in d]}")
        key = (d[0], a[1], a[2])
        best = min(NOMINAL_DH, key=lambda m: sum(abs(x - y) for x, y in zip(key, NOMINAL_DH[m])))
        err = sum(abs(x - y) for x, y in zip(key, NOMINAL_DH[best]))
        print(f"  -> arm model from DH parameters: {best} (deviation {err * 1000:.1f} mm)")
    else:
        print("  no kinematics info seen")

    if 1 in state and len(state[1]) >= 6 * 41:                  # joint data
        q = [struct.unpack_from(">d", state[1], 41 * j)[0] for j in range(6)]
        dist = max(abs(x - y) for x, y in zip(q, HOME_JOINTS))
        print(f"  joints now [rad] = {[round(x, 4) for x in q]}  (max {dist:.3f} rad from the plan's home)")

    if 2 in state and len(state[2]) >= 32:                      # tool data
        _, _, ai0, ai1, v48, vout, cur, temp, mode = struct.unpack_from(">bbddfBffB", state[2])
        print(f"  tool flange: output voltage {vout} V, current {cur * 1000:.0f} mA, "
              f"temperature {temp:.1f} C, analog in {ai0:.2f} / {ai1:.2f}")
        if vout and cur > 0.05:
            print("  -> the tool flange powers a device: a gripper cabled to the flange "
                  "(older RG6 URCap) or a Quick Changer for I/O")
        else:
            print("  -> nothing powered from the tool flange (fits an RG6 powered by a Compute Box)")

    if 3 in state and len(state[3]) >= 8:                       # masterboard
        din, dout = struct.unpack_from(">ii", state[3])
        print(f"  digital in bits 0x{din & 0xffffffff:08x}, out bits 0x{dout & 0xffffffff:08x} "
              "(bits 16-17 = tool I/O)")


def modbus_read(ip, unit, address, count, timeout=2.0):
    with socket.create_connection((ip, 502), timeout=timeout) as s:
        s.sendall(struct.pack(">HHHBBHH", 1, 0, 6, unit, 0x03, address, count))
        hdr = s.recv(9)
        if len(hdr) < 9 or hdr[7] != 0x03:
            raise OSError(f"Modbus exception (function 0x{hdr[7]:02x}, code {hdr[8]})"
                          if len(hdr) >= 9 else "short reply")
        data = b""
        while len(data) < hdr[8]:
            chunk = s.recv(hdr[8] - len(data))
            if not chunk:
                break
            data += chunk
        return struct.unpack(f">{len(data) // 2}H", data)


def port_open(ip, port, timeout=0.5):
    try:
        socket.create_connection((ip, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def compute_box(candidates, robot_ip):
    print("\n== OnRobot Compute Box (Modbus TCP, unit 65, registers 258-275)")
    found = False
    for ip in candidates:
        if ip == robot_ip:
            continue                    # the UR controller runs its own Modbus server
        if not port_open(ip, 502, 1.0):
            print(f"  {ip}: port 502 closed or unreachable")
            continue
        try:
            st = modbus_read(ip, 65, 258, 18)
        except OSError as e:
            print(f"  {ip}: Modbus server, but no RG gripper at unit 65 ({e})")
            continue
        found = True
        status = st[10]
        print(f"  {ip}: RG gripper answers. width {st[17] / 10:.1f} mm, status 0x{status:04x} "
              f"(busy {status & 1}, grip detected {(status >> 1) & 1})")
        print(f"         registers 258-275 = {list(st)}")
    if not found:
        print("  -> no Compute Box with an RG gripper found")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot_ip")
    ap.add_argument("--compute-box", nargs="*", default=[], help="extra addresses to try for the Compute Box")
    ap.add_argument("--scan", action="store_true", help="look for Modbus servers (port 502) on the robot's /24")
    args = ap.parse_args()

    dashboard(args.robot_ip)
    primary(args.robot_ip)

    candidates = ["192.168.1.1"] + args.compute_box
    if args.scan:
        net = ipaddress.ip_network(f"{args.robot_ip}/24", strict=False)
        hosts = [str(h) for h in net.hosts()]
        with ThreadPoolExecutor(64) as ex:
            hits = [h for h, ok in zip(hosts, ex.map(lambda h: port_open(h, 502), hosts)) if ok]
        print(f"\n== Hosts with port 502 open on {net}: {hits or 'none'}")
        candidates += hits
    compute_box(list(dict.fromkeys(candidates)), args.robot_ip)


if __name__ == "__main__":
    main()
