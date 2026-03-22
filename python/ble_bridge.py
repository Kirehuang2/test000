"""
BLE-to-UDP bridge: receives packed data from DA14531 via bleak,
forwards decoded samples to MATLAB via UDP on localhost:5000.

UDP packet format (one per sample):
  "timestamp,rpm,iq,vbat,tinv,treg\n"

Usage: python ble_bridge.py [hz]
  hz: sampling rate (default 100)
"""
import asyncio
import socket
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "SPS_531"
SERVER_TX_UUID = "0783b03e-8535-b5a0-7140-a304d2495cb8"
SERVER_RX_UUID = "0783b03e-8535-b5a0-7140-a304d2495cba"
FLOW_CTRL_UUID = "0783b03e-8535-b5a0-7140-a304d2495cb9"

UDP_HOST = "127.0.0.1"
UDP_PORT = 5000

BURST_SIZE = 1
sample_count = 0
msg_count = 0
burst_count = 0
restart_needed = False
rx_buf = bytearray()
last_rx_time = 0
udp_sock = None
start_time_s = 0  # monotonic start

def ts():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

def on_notify(sender, data: bytearray):
    global rx_buf, sample_count, msg_count, burst_count, restart_needed, last_rx_time
    rx_buf.extend(data)
    while b'\n' in rx_buf:
        idx = rx_buf.index(b'\n')
        line = rx_buf[:idx].decode('ascii', errors='replace').strip()
        rx_buf = rx_buf[idx+1:]

        if not line or ',' not in line:
            continue

        last_rx_time = asyncio.get_event_loop().time()

        if line.startswith('B'):
            parts = line[1:].split(',')
            n = int(parts[0])
            rpms = parts[1:1+n]
            extras = parts[1+n:]  # iq, vbat, tinv, treg
            msg_count += 1
            burst_count += 1

            iq = extras[0] if len(extras) > 0 else "0"
            vbat = extras[1] if len(extras) > 1 else "0"
            tinv = extras[2] if len(extras) > 2 else "0"
            treg = extras[3] if len(extras) > 3 else "0"

            t_now = asyncio.get_event_loop().time() - start_time_s
            for i, rpm in enumerate(rpms):
                sample_count += 1
                # Send each sample via UDP
                # Estimate timestamp: spread samples evenly within pack period
                pack_period = n * 0.01 if n > 0 else 0.08  # default 10ms/sample
                t_sample = t_now - pack_period + (i + 1) * (pack_period / n)
                pkt = f"{t_sample:.3f},{rpm},{iq},{vbat},{tinv},{treg}\n"
                try:
                    udp_sock.sendto(pkt.encode(), (UDP_HOST, UDP_PORT))
                except Exception:
                    pass

            print(f"[{ts()}] B{n} msg#{msg_count} → {n} UDP pkts (total {sample_count})")

            if burst_count >= BURST_SIZE:
                burst_count = 0
                restart_needed = True

        elif line.startswith('L'):
            msg_count += 1
            sample_count += 1
            burst_count += 1
            parts = line[1:].split(',')
            t_now = asyncio.get_event_loop().time() - start_time_s
            rpm = parts[0] if len(parts) > 0 else "0"
            iq = parts[1] if len(parts) > 1 else "0"
            vbat = parts[2] if len(parts) > 2 else "0"
            tinv = parts[3] if len(parts) > 3 else "0"
            treg = parts[4] if len(parts) > 4 else "0"
            pkt = f"{t_now:.3f},{rpm},{iq},{vbat},{tinv},{treg}\n"
            try:
                udp_sock.sendto(pkt.encode(), (UDP_HOST, UDP_PORT))
            except Exception:
                pass

            if burst_count >= BURST_SIZE:
                burst_count = 0
                restart_needed = True

def on_flow(sender, data: bytearray):
    val = data[0] if data else -1
    label = "XON" if val == 1 else "XOFF" if val == 0 else f"0x{val:02X}"
    print(f"[{ts()}] FLOW< {label}")

async def send_restart(client):
    for _ in range(3):
        await client.write_gatt_char(SERVER_RX_UUID, b"R\r\n", response=False)
        await asyncio.sleep(0.05)

async def main():
    global sample_count, msg_count, burst_count, restart_needed, last_rx_time
    global udp_sock, start_time_s

    hz = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    pack_time = 8.0 / hz  # seconds to fill one pack

    # Setup UDP socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"UDP bridge: sending to {UDP_HOST}:{UDP_PORT}")
    print(f"Sampling rate: {hz}Hz, pack time: {pack_time:.3f}s")

    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if not device:
        print("Device not found!")
        return

    async with BleakClient(device, timeout=15) as client:
        print(f"Connected. MTU={client.mtu_size}")

        await client.start_notify(SERVER_TX_UUID, on_notify)
        await client.start_notify(FLOW_CTRL_UUID, on_flow)
        await asyncio.sleep(0.5)
        await client.write_gatt_char(FLOW_CTRL_UUID, bytearray([0x01]), response=False)
        await asyncio.sleep(1)

        # Initial start with retries
        wait_time = max(pack_time + 2.0, 3.0)
        cmd = f"LOG START {hz}\r\n".encode()
        for attempt in range(5):
            await client.write_gatt_char(SERVER_RX_UUID, cmd, response=False)
            print(f"[{ts()}] TX> LOG START {hz} (attempt {attempt+1})")
            await asyncio.sleep(wait_time)
            if sample_count > 0:
                break

        if sample_count == 0:
            print("Failed to start.")
            return

        start_time_s = asyncio.get_event_loop().time() - (asyncio.get_event_loop().time() - last_rx_time)
        restart_count = 0
        fallback_timeout = max(pack_time + 4.0, 3.0)
        print(f"--- Streaming (Ctrl+C to stop, timeout={fallback_timeout:.1f}s) ---")

        try:
            while True:
                await asyncio.sleep(0.05)
                now = asyncio.get_event_loop().time()

                if restart_needed:
                    restart_needed = False
                    restart_count += 1
                    await send_restart(client)
                    last_rx_time = now

                elif (now - last_rx_time) > fallback_timeout:
                    restart_count += 1
                    print(f"[{ts()}] timeout, restart #{restart_count}")
                    await send_restart(client)
                    last_rx_time = now

        except KeyboardInterrupt:
            pass

        await client.write_gatt_char(SERVER_RX_UUID, b"S\r\n", response=False)
        await asyncio.sleep(0.3)

        elapsed = asyncio.get_event_loop().time() - start_time_s
        rate = sample_count / elapsed if elapsed > 0 else 0
        print(f"\n=== {sample_count} samples ({msg_count} msgs) in {elapsed:.0f}s ({rate:.2f}/s, {restart_count} restarts) ===")

    udp_sock.close()
    print("Disconnected.")

if __name__ == "__main__":
    asyncio.run(main())
