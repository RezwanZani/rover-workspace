#!/usr/bin/env python3
import os
import pty
import socket
import select
import sys
import tty
import termios

VIRTUAL_TTY = "/tmp/ttyUSB0"
LISTEN_PORT = 9870

# 1. Allocate pseudo-terminal
master_fd, slave_fd = pty.openpty()
slave_name = os.ttyname(slave_fd)

# 2. Put PTY slave into RAW mode immediately (disables canonical buffering & echo)
tty.setraw(slave_fd)

# 3. Create persistent symlink
if os.path.exists(VIRTUAL_TTY) or os.path.islink(VIRTUAL_TTY):
    os.unlink(VIRTUAL_TTY)
os.symlink(slave_name, VIRTUAL_TTY)
os.chmod(VIRTUAL_TTY, 0o666)

print(f"[PTY BRIDGE] Virtual raw serial port active at {VIRTUAL_TTY} -> {slave_name}")

# 4. Bind persistent TCP listener
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', LISTEN_PORT))
server.listen(1)
print(f"[PTY BRIDGE] Waiting for Windows bridge on port {LISTEN_PORT}...")

try:
    while True:
        conn, addr = server.accept()
        print(f"[PTY BRIDGE] Windows host connected from {addr}!")
        conn.setblocking(False)

        while True:
            r_fds, _, _ = select.select([conn, master_fd], [], [], 0.02)

            # Windows TCP -> WSL PTY
            if conn in r_fds:
                try:
                    data = conn.recv(1024)
                    if not data:
                        print("[PTY BRIDGE] Windows disconnected. Waiting for reconnect...")
                        break
                    os.write(master_fd, data)
                    print(f"[BRIDGE] Routed {len(data)} bytes to PTY", flush=True)
                except (ConnectionResetError, BlockingIOError):
                    break

            # WSL PTY -> Windows TCP
            if master_fd in r_fds:
                try:
                    data = os.read(master_fd, 1024)
                    if data:
                        conn.sendall(data)
                except (BlockingIOError, OSError):
                    pass

        conn.close()

except KeyboardInterrupt:
    print("\n[PTY BRIDGE] Shutting down...")
finally:
    if os.path.islink(VIRTUAL_TTY):
        os.unlink(VIRTUAL_TTY)
    os.close(master_fd)
    os.close(slave_fd)
    server.close()