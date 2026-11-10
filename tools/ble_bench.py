#!/usr/bin/env python3
"""Propeller-off BLE diagnostics. Deliberately cannot arm the flight controller.

python tools/ble_bench.py --scan
python tools/ble_bench.py --address <address-or-macOS-UUID>
python tools/ble_bench.py --address <id> --motor 1 --pulse 1100
"""
import argparse
import asyncio
import binascii
import struct
import time

SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
ARM, KILL, TEST, DEBUG = 1, 2, 4, 8


def packet(sequence, roll=0, pitch=0, yaw=0, throttle=1000, flags=0):
    if not all(-1000 <= v <= 1000 for v in (roll, pitch, yaw)):
        raise ValueError("Stick values must be -1000..1000")
    if not 1000 <= throttle <= 2000 or flags & ~15:
        raise ValueError("Invalid throttle or flags")
    if flags & TEST and (flags & ARM or roll not in range(1, 5) or
                         pitch or yaw or throttle > 1150):
        raise ValueError("Invalid motor test")
    body = struct.pack("<BBIhhhH", 0xA1, flags, sequence & 0xFFFFFFFF,
                       roll, pitch, yaw, throttle)
    return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


async def run(args):
    from bleak import BleakClient, BleakScanner

    if args.scan:
        devices = await BleakScanner.discover(timeout=8, service_uuids=[SERVICE])
        for device in devices:
            print(f"{device.address}  {device.name}")
        return
    sequence = 0
    async with BleakClient(args.address) as client:
        print("Connected. This tool cannot ARM. Remove all propellers.")
        await client.start_notify(TX, lambda _, data: print(data.decode(errors="replace")))

        async def send(**kwargs):
            nonlocal sequence
            sequence = (sequence + 1) & 0xFFFFFFFF
            await client.write_gatt_char(RX, packet(sequence, **kwargs), response=True)

        try:
            # Explicit release before test; a test packet as the first packet is refused.
            for _ in range(10):
                await send()
                await asyncio.sleep(0.02)
            if args.debug:
                await send(flags=DEBUG)
            elif args.motor:
                print(f"Testing motor {args.motor} at {args.pulse} us, maximum 2 s.")
                started = time.monotonic()
                while time.monotonic() - started < 1.8:
                    await send(roll=args.motor, throttle=args.pulse, flags=TEST)
                    await asyncio.sleep(0.02)
            else:
                print("Disarmed status session; waiting five seconds.")
                for _ in range(250):
                    await send()
                    await asyncio.sleep(0.02)
        finally:
            # A disconnected client also trips the firmware timeout. Best effort stop.
            if client.is_connected:
                try:
                    await asyncio.wait_for(send(flags=KILL), timeout=0.2)
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scan", action="store_true")
    group.add_argument("--address")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--motor", type=int, choices=range(1, 5))
    action.add_argument("--debug", action="store_true")
    parser.add_argument("--pulse", type=int, default=1100, choices=range(1000, 1151))
    args = parser.parse_args()
    if args.scan and (args.motor or args.debug):
        parser.error("Motor/debug commands require --address")
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
