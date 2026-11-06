"""
Zero-Dependency Non-Blocking WebSocket (RFC 6455) & Static HTTP Telemetry Server.
- Works out-of-the-box with Python's standard library (no external 'websockets' pip package required).
- Serves pc_dashboard/ at http://<host>:<port>/ and accepts WebSocket connections on ws://<host>:<port>/.
- Broadcasts Section 4B JSON state updates and handles zero connected clients gracefully.
"""

import asyncio
import base64
import hashlib
import json
import logging
from pathlib import Path
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional, Set

logger = logging.getLogger("gantry.telemetry")

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class TelemetryServer:
    """Background asyncio HTTP + RFC 6455 WebSocket server using pure Python stdlib."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8765,
        command_callback: Optional[Callable[[str, dict], None]] = None,
    ):
        self.host = host
        self.port = port
        self.command_callback = command_callback

        self._clients: Set[asyncio.StreamWriter] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._latest_payload: Dict[str, Any] = {
            "timestamp": time.time(),
            "state": "IDLE",
            "gantry_pose": {
                "x_mm": 49.1,
                "y_mm": 60.1,
                "z_mm": 0.0,
                "angle_deg": 0.0,
                "grip_closed": True,
            },
            "calibration": {
                "calibrated": False,
                "kx_mm_per_px": 0.25,
                "ky_mm_per_px": 0.25,
            },
            "blocks": [],
            "logs": [],
            "vision_frame_b64": None,
        }
        self._dashboard_dir = Path(__file__).resolve().parent.parent / "pc_dashboard"

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._thread.start()
        logger.info(
            "TelemetryServer listening on http://%s:%d/ and ws://%s:%d/",
            self.host,
            self.port,
            self.host,
            self.port,
        )

    def broadcast_state(self, state_dict: Dict[str, Any]) -> None:
        """Non-blocking state broadcast; safe with zero connected WebSocket clients."""
        self._latest_payload = state_dict
        if not self._clients or self._loop is None or not self._loop.is_running():
            return

        msg = json.dumps(state_dict)
        asyncio.run_coroutine_threadsafe(self._async_broadcast(msg), self._loop)

    async def _async_broadcast(self, message: str) -> None:
        if not self._clients:
            return
        frame = self._encode_ws_text_frame(message)
        stale: Set[asyncio.StreamWriter] = set()
        for writer in list(self._clients):
            try:
                writer.write(frame)
                await writer.drain()
            except Exception:
                stale.add(writer)
        for w in stale:
            self._clients.discard(w)
            try:
                w.close()
            except Exception:
                pass

    def _run_event_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self) -> None:
        server = await asyncio.start_server(
            self._handle_connection, self.host, self.port
        )
        async with server:
            await server.serve_forever()

    async def _handle_connection(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            header_bytes = await reader.readuntil(b"\r\n\r\n")
        except Exception:
            writer.close()
            return

        header_text = header_bytes.decode("utf-8", errors="replace")
        lines = header_text.split("\r\n")
        if not lines:
            writer.close()
            return

        req_parts = lines[0].split(" ")
        path = req_parts[1] if len(req_parts) > 1 else "/"

        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        # Check if this is a WebSocket Upgrade handshake
        if headers.get("upgrade", "").lower() == "websocket":
            ws_key = headers.get("sec-websocket-key", "")
            accept_raw = hashlib.sha1((ws_key + WS_GUID).encode("ascii")).digest()
            accept_key = base64.b64encode(accept_raw).decode("ascii")

            handshake = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
            )
            writer.write(handshake.encode("ascii"))
            await writer.drain()
            await self._handle_websocket_client(reader, writer)
            return

        # Otherwise serve static HTTP files for pc_dashboard
        await self._handle_http_request(path, writer)

    async def _handle_http_request(
        self, path: str, writer: asyncio.StreamWriter
    ) -> None:
        clean_path = path.split("?", 1)[0]
        if clean_path in ("/", "/index.html"):
            target = self._dashboard_dir / "index.html"
            ctype = "text/html; charset=utf-8"
        elif clean_path == "/styles.css":
            target = self._dashboard_dir / "styles.css"
            ctype = "text/css; charset=utf-8"
        elif clean_path == "/app.js":
            target = self._dashboard_dir / "app.js"
            ctype = "application/javascript; charset=utf-8"
        elif clean_path == "/api/state":
            body = json.dumps(self._latest_payload).encode("utf-8")
            resp = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                f"Content-Length: {len(body)}\r\n"
                "Connection: close\r\n\r\n"
            ).encode("ascii") + body
            writer.write(resp)
            await writer.drain()
            writer.close()
            return
        else:
            writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\nConnection: close\r\n\r\nNot Found")
            await writer.drain()
            writer.close()
            return

        if target.exists():
            body = target.read_bytes()
            resp = (
                "HTTP/1.1 200 OK\r\n"
                f"Content-Type: {ctype}\r\n"
                f"Content-Length: {len(body)}\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n"
            ).encode("ascii") + body
            writer.write(resp)
        else:
            writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\nConnection: close\r\n\r\nNot Found")

        await writer.drain()
        writer.close()

    async def _handle_websocket_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._clients.add(writer)
        logger.info("Tier 1 Dashboard client connected (%d active)", len(self._clients))
        try:
            # Push initial snapshot
            writer.write(self._encode_ws_text_frame(json.dumps(self._latest_payload)))
            await writer.drain()

            while True:
                msg = await self._read_ws_frame(reader)
                if msg is None:
                    break
                try:
                    data = json.loads(msg)
                except json.JSONDecodeError:
                    data = {"command": msg.strip()}

                cmd = str(data.get("command", "")).upper().strip()
                if cmd and self.command_callback:
                    self.command_callback(cmd, data)
        except Exception as exc:
            logger.debug("WebSocket client disconnected: %s", exc)
        finally:
            self._clients.discard(writer)
            try:
                writer.close()
            except Exception:
                pass

    @staticmethod
    def _encode_ws_text_frame(text: str) -> bytes:
        payload = text.encode("utf-8")
        length = len(payload)
        header = bytearray([0x81])  # FIN=1, opcode=1 (text)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))
        return bytes(header) + payload

    @staticmethod
    async def _read_ws_frame(reader: asyncio.StreamReader) -> Optional[str]:
        hdr = await reader.readexactly(2)
        opcode = hdr[0] & 0x0F
        if opcode == 0x8:  # Close frame
            return None
        masked = (hdr[1] & 0x80) != 0
        length = hdr[1] & 0x7F
        if length == 126:
            ext = await reader.readexactly(2)
            length = struct.unpack("!H", ext)[0]
        elif length == 127:
            ext = await reader.readexactly(8)
            length = struct.unpack("!Q", ext)[0]

        mask_key = await reader.readexactly(4) if masked else b""
        data = bytearray(await reader.readexactly(length))
        if masked:
            for i in range(length):
                data[i] ^= mask_key[i % 4]
        return data.decode("utf-8", errors="replace")
