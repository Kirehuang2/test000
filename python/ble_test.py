"""
BLE streaming with packed data + burst restart
Packed format: B8,rpm1,rpm2,...,rpm8,iq,vbat,tinv,treg
Legacy format: L rpm,iq,vbat,tinv,treg
"""
import asyncio
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "SPS_531"
SERVER_TX_UUID = "0783b03e-8535-b5a0-7140-a304d2495cb8"
SERVER_RX_UUID = "0783b03e-8535-b5a0-7140-a304d2495cba"
FLOW_CTRL_UUID = "0783b03e-8535-b5a0-7140-a304d2495cb9"

BURST_SIZE = 1          # MCU sends 1 packed message then auto-stops
sample_count = 0        # total RPM samples received
msg_count = 0           # total BLE messages received
burst_count = 0         # messages in current burst
restart_needed = False
rx_buf = bytearray()
last_rx_time = 0

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
            if line:
                print(f"[{ts()}] ?? {line}")
            continue

        last_rx_time = asyncio.get_event_loop().time()

        if line.startswith('B'):
            # Packed: B8,r1,r2,...,r8,iq,vbat,tinv,treg
            parts = line[1:].split(',')
            n = int(parts[0])
            rpms = parts[1:1+n]
            extras = parts[1+n:]  # iq, vbat, tinv, treg
            msg_count += 1
            burst_count += 1
            for i, rpm in enumerate(rpms):
                sample_count += 1
                extra_str = f"  iq={extras[0]} V={extras[1]} Ti={extras[2]} Tr={extras[3]}" if i == n-1 and len(extras) >= 4 else ""
                print(f"[{ts()}] #{sample_count:4d} RPM={rpm:>6s}{extra_str}  (msg {msg_count})")
            if burst_count >= BURST_SIZE:
                burst_count = 0
                restart_needed = True

        elif line.startswith('L'):
            # Legacy: L rpm,iq,vbat,tinv,treg
            msg_count += 1
            sample_count += 1
            burst_count += 1
            print(f"[{ts()}] #{sample_count:4d} {line}  (msg {msg_count})")
            if burst_count >= BURST_SIZE:
                burst_count = 0
                restart_needed = True
        else:
            print(f"[{ts()}] ?? {line}")

def on_flow(sender, data: bytearray):
    val = data[0] if data else -1
    label = "XON" if val == 1 else "XOFF" if val == 0 else f"0x{val:02X}"
    print(f"[{ts()}] FLOW< {label}")

async def send_restart(client):
    """Send triple-R for redundancy."""
    for _ in range(3):
        await client.write_gatt_char(SERVER_RX_UUID, b"R\r\n", response=False)
        await asyncio.sleep(0.05)

async def main():
    global sample_count, msg_count, burst_count, restart_needed, last_rx_time

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

        # Initial start with retries (10s wait for packed mode: 8 samples at 1Hz)
        for attempt in range(3):
            await client.write_gatt_char(SERVER_RX_UUID, b"LOG START 1\r\n", response=False)
            print(f"[{ts()}] TX> LOG START 1 (attempt {attempt+1}) — waiting 10s for packed data...")
            await asyncio.sleep(10)
            if sample_count > 0:
                break

        if sample_count == 0:
            print("Failed to start.")
            return

        last_rx_time = asyncio.get_event_loop().time()
        restart_count = 0
        print(f"--- Streaming 120s (Enter to stop) ---")
        print(f"--- Packed mode: 8 samples/msg, 1 msg/burst, instant restart ---")

        stop = False
        async def wait_input():
            nonlocal stop
            await asyncio.get_event_loop().run_in_executor(None, input)
            stop = True

        input_task = asyncio.create_task(wait_input())
        start_time = asyncio.get_event_loop().time()

        while not stop:
            await asyncio.sleep(0.1)
            now = asyncio.get_event_loop().time()

            if restart_needed:
                restart_needed = False
                restart_count += 1
                print(f"[{ts()}] --- burst done, restart #{restart_count} ---")
                await send_restart(client)
                last_rx_time = now

            elif (now - last_rx_time) > 12.0:
                # Fallback: 8s accumulation + 4s margin
                restart_count += 1
                print(f"[{ts()}] --- silence timeout, restart #{restart_count} ---")
                await send_restart(client)
                last_rx_time = now

            if now - start_time > 120:
                break

        input_task.cancel()
        await client.write_gatt_char(SERVER_RX_UUID, b"S\r\n", response=False)
        await asyncio.sleep(0.5)

        elapsed = asyncio.get_event_loop().time() - start_time
        rate = sample_count / elapsed if elapsed > 0 else 0
        print(f"\n=== {sample_count} samples ({msg_count} msgs) in {elapsed:.0f}s ({rate:.2f} samples/s, {restart_count} restarts) ===")

    print("Disconnected.")

if __name__ == "__main__":
    asyncio.run(main())
