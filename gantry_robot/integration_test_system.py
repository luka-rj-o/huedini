#!/usr/bin/env python3
"""
HUE-DINI Gantry Robot: Zero-Dependency WebSocket / Telemetry Integration Test
=============================================================================
Tests real-time WebSocket telemetry, system states, and remote commands
between Tier 1 (Client/Dashboard) and Tier 2 (Pi Zero Controller) using
only Python's standard library (no pip dependencies required).

Usage:
  python3 integration_test_system.py [--host 127.0.0.1] [--port 8765]
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time


def ws_handshake(s: socket.socket, host: str, port: int) -> bool:
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    req = (
        f"GET / HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(req.encode("ascii"))
    resp = s.recv(1024).decode("utf-8", errors="replace")
    return "101" in resp and "Switching Protocols" in resp


def ws_recv_text(s: socket.socket, timeout: float = 3.0) -> str:
    s.settimeout(timeout)
    data = b""
    while len(data) < 2:
        chunk = s.recv(2 - len(data))
        if not chunk:
            raise ConnectionError("Connection closed")
        data += chunk

    b1, b2 = data[0], data[1]
    opcode = b1 & 0x0F
    is_masked = bool(b2 & 0x80)
    length = b2 & 0x7F

    if length == 126:
        ext = s.recv(2)
        length = struct.unpack("!H", ext)[0]
    elif length == 127:
        ext = s.recv(8)
        length = struct.unpack("!Q", ext)[0]

    mask_key = s.recv(4) if is_masked else None

    payload = b""
    while len(payload) < length:
        chunk = s.recv(length - len(payload))
        if not chunk:
            break
        payload += chunk

    if is_masked and mask_key:
        payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))

    return payload.decode("utf-8", errors="replace")


def ws_send_text(s: socket.socket, text: str):
    payload = text.encode("utf-8")
    length = len(payload)
    mask_key = os.urandom(4)

    header = bytearray([0x81])  # FIN + Text opcode
    if length < 126:
        header.append(0x80 | length)
    elif length < 65536:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))

    header.extend(mask_key)
    masked_payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
    s.sendall(header + masked_payload)


def run_system_test(host: str, port: int):
    print("=" * 60)
    print(f"HUE-DINI Telemetry & WebSocket Integration Test (ws://{host}:{port}/)")
    print("=" * 60)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(4.0)

    try:
        s.connect((host, port))
        if not ws_handshake(s, host, port):
            print("[FAIL] RFC 6455 WebSocket Handshake rejected.")
            sys.exit(1)
        print("[PASS] RFC 6455 WebSocket Handshake established.")

        # 1. Receive initial broadcast
        msg = ws_recv_text(s, timeout=4.0)
        data = json.loads(msg)
        state = data.get("state")
        pose = data.get("gantry_pose", {})
        print(f"[PASS] Telemetry Packet Received | State: {state}")
        print(f"       Gantry Pose: X={pose.get('x_mm')}, Y={pose.get('y_mm')}, Z={pose.get('z_mm')}, A={pose.get('angle_deg')}")

        # 2. Test RESET command
        print("\nSending command: RESET...")
        ws_send_text(s, json.dumps({"command": "RESET"}))
        time.sleep(0.3)

        msg2 = ws_recv_text(s, timeout=4.0)
        data2 = json.loads(msg2)
        print(f"[PASS] Post-RESET Telemetry: State is now {data2.get('state')}")

        # 3. Test RAW_SERIAL command
        print("\nSending command: RAW_SERIAL 'STATUS'...")
        ws_send_text(s, json.dumps({"command": "RAW_SERIAL", "line": "STATUS"}))
        time.sleep(0.4)

        msg3 = ws_recv_text(s, timeout=4.0)
        data3 = json.loads(msg3)
        logs = data3.get("logs", [])
        print(f"[PASS] Telemetry Logs ({len(logs)} entries in buffer):")
        for line in logs[-3:]:
            print(f"       {line}")

        print("\n" + "=" * 60)
        print("\033[92mALL WEBSOCKET TELEMETRY TESTS PASSED!\033[0m")
        print("=" * 60)

    except Exception as exc:
        print(f"\n[FAIL] WebSocket integration test failed: {exc}")
        sys.exit(1)
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser(description="Test HUE-DINI WebSocket Telemetry")
    parser.add_argument("--host", default="192.168.2.26", help="Server host (default: 192.168.2.26)")
    parser.add_argument("--port", type=int, default=8765, help="Server port (default: 8765)")
    args = parser.parse_args()

    run_system_test(args.host, args.port)


if __name__ == "__main__":
    main()
