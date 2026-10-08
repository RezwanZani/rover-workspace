#!/usr/bin/env python3
import serial
import struct
import sys
import time

SERIAL_PORT = "/tmp/ttyUSB0"
BAUD_RATE = 115200
PACKET_FORMAT = '<cffb'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # Exactly 10 bytes

print(f"[WSL TEST] Connecting to virtual port: {SERIAL_PORT}...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[WSL TEST] Connected successfully! Waiting for incoming packets...")
except serial.SerialException as e:
    print(f"[WSL TEST ERROR] Failed to open {SERIAL_PORT}: {e}")
    print("Ensure socat is actively running in another WSL terminal.")
    sys.exit(1)

buffer = bytearray()

try:
    while True:
        try:
            waiting = ser.in_waiting
        except OSError:
            print("\n[WSL TEST ERROR] Port disconnected.")
            break

        if waiting > 0:
            try:
                data = ser.read(waiting)
            except serial.SerialException as e:
                print(f"\n[WSL TEST ERROR] {e}")
                break
                
            buffer.extend(data)

            # Slide window to find sync marker and extract complete 10-byte packets
            while len(buffer) >= PACKET_SIZE:
                # Re-align buffer if sync character '$' is displaced
                if buffer[0:1] != b'$':
                    buffer.pop(0)
                    continue

                packet = bytes(buffer[:PACKET_SIZE])
                del buffer[:PACKET_SIZE]

                try:
                    sync_char, lin_x, ang_z, cmd_flag = struct.unpack(PACKET_FORMAT, packet)

                    print(f"[RX OK] Sync: {sync_char.decode('ascii', errors='ignore')} | "
                          f"Lin_X: {lin_x:>6.2f} m/s | "
                          f"Ang_Z: {ang_z:>6.2f} rad/s | "
                          f"Flag: {cmd_flag}")

                    # Echo acknowledgment back across the bridge for ping monitoring
                    ack_packet = struct.pack('<cb', b'#', cmd_flag)
                    ser.write(ack_packet)

                except struct.error as e:
                    print(f"[UNPACK ERROR] {e}")

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n[WSL TEST] Stopped by user.")
finally:
    ser.close()