"""UDP receiver mirroring SerialReceiver's contract.

Consumes raw UDP datagrams from the coordinator's WiFi forward path.
Unlike SerialReceiver, no COBS decode is performed — UDP is already
framed, so each datagram is exactly one frame.
"""

from __future__ import annotations

import socket
from typing import Generator


class UDPReceiver:
    """UDP listener yielding raw frames matching SerialReceiver.read_packets()."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 4211,
        recv_buffer_size: int = 65536,
    ) -> None:
        self._bind_host = bind_host
        self._bind_port = bind_port
        self._recv_buffer_size = recv_buffer_size
        self._sock: socket.socket | None = None

    def open(self) -> None:
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Raise the kernel receive buffer so bursty capture doesn't drop
        # datagrams between our recvfrom() calls.
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._recv_buffer_size)
        # Non-blocking with a short timeout so Ctrl-C breaks the read loop.
        s.settimeout(0.5)
        s.bind((self._bind_host, self._bind_port))
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def read_packets(self) -> Generator[bytes, None, None]:
        """Yield raw UDP payloads indefinitely (until close() is called).

        Each yielded bytes object IS one frame. Do NOT apply COBS decode --
        that's a USB-transport concern; UDP is already framed.
        """
        if self._sock is None:
            raise RuntimeError("UDPReceiver not open -- call open() first")
        while True:
            try:
                payload, _addr = self._sock.recvfrom(2100)
                if payload:
                    yield payload
            except socket.timeout:
                # Natural yield point so the caller's loop can check
                # termination conditions (Ctrl-C, elapsed --seconds, etc.).
                continue
            except OSError:
                # Socket closed underneath us -- terminate generator cleanly.
                break
