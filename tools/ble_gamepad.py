#!/usr/bin/env python3
"""Experimental BLE pilot bridge. Requires a gamepad and explicit control mapping.

Install tools/requirements-pilot.txt, then use --inspect to identify axes/buttons.
Both the ARM and deadman buttons must remain held; releasing either disarms.
No reconnection or autonomous flight. Validate every mapping with props removed.
"""
import argparse
import asyncio
import time

from ble_bench import ARM, KILL, RX, TX, packet


def sticks(joystick, args):
    def axis(index, sign):
        v = max(-1.0, min(1.0, joystick.get_axis(index) * sign))
        return v

    values = [axis(args.roll_axis, args.roll_sign),
              axis(args.pitch_axis, args.pitch_sign),
              axis(args.yaw_axis, args.yaw_sign)]
    # Map the full analog throttle axis: -1 -> 1000 us, +1 -> 2000 us.
    throttle = round(1500 + 500 * axis(args.throttle_axis, args.throttle_sign))
    control = [round(v * 1000) if abs(v) > 0.05 else 0 for v in values]
    return (*control, throttle)


async def pilot(args):
    import pygame

    pygame.init()
    pygame.joystick.init()
    if not pygame.joystick.get_count():
        raise RuntimeError("No gamepad found")
    joystick = pygame.joystick.Joystick(args.gamepad)
    joystick.init()
    print(joystick.get_name(), "axes:", joystick.get_numaxes(),
          "buttons:", joystick.get_numbuttons())
    if args.inspect:
        while True:
            pygame.event.pump()
            print("axes", [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())],
                  "buttons", [i for i in range(joystick.get_numbuttons()) if joystick.get_button(i)])
            await asyncio.sleep(0.2)

    axes = (args.roll_axis, args.pitch_axis, args.yaw_axis, args.throttle_axis)
    if len(set(axes)) != 4 or any(i < 0 or i >= joystick.get_numaxes() for i in axes):
        raise ValueError("Choose four different valid axes")
    if args.arm_button == args.deadman_button or any(
            i < 0 or i >= joystick.get_numbuttons() for i in (args.arm_button, args.deadman_button)):
        raise ValueError("Choose two different valid buttons")

    from bleak import BleakClient
    async with BleakClient(args.address) as client:
        await client.start_notify(TX, lambda _, value: print(value.decode(errors="replace")))
        print("Connected. Start with both buttons RELEASED and throttle fully LOW.")
        print("Hold both buttons for ARM; releasing either cuts all motors.")
        sequence = 0
        released = False
        async def send(*, controls=(0, 0, 0, 1000), flags=0):
            nonlocal sequence
            sequence = (sequence + 1) & 0xFFFFFFFF
            await asyncio.wait_for(client.write_gatt_char(
                RX, packet(sequence, *controls, flags=flags), response=True), timeout=0.15)

        try:
            last = time.monotonic()
            while client.is_connected:
                events = pygame.event.get()
                if any(e.type == pygame.JOYDEVICEREMOVED and
                       e.instance_id == joystick.get_instance_id() for e in events):
                    raise RuntimeError("Gamepad disconnected")
                now = time.monotonic()
                if now-last > 0.15:
                    raise RuntimeError("Control loop stalled")
                last = now
                controls = sticks(joystick, args)
                arm = bool(joystick.get_button(args.arm_button))
                deadman = bool(joystick.get_button(args.deadman_button))
                if not arm and not deadman and controls[3] <= 1050:
                    released = True
                if released and arm and deadman:
                    await send(controls=controls, flags=ARM)
                else:
                    await send(flags=0)
                await asyncio.sleep(0.02)
        finally:
            if client.is_connected:
                try:
                    await send(flags=KILL)
                except Exception:
                    pass
            pygame.quit()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inspect", action="store_true")
    p.add_argument("--address")
    p.add_argument("--gamepad", type=int, default=0)
    for name in ("roll", "pitch", "yaw", "throttle"):
        p.add_argument(f"--{name}-axis", type=int)
        p.add_argument(f"--{name}-sign", type=int, choices=(-1, 1), default=1)
    p.add_argument("--arm-button", type=int)
    p.add_argument("--deadman-button", type=int)
    args = p.parse_args()
    if not args.inspect and any(v is None for v in
            (args.address, args.roll_axis, args.pitch_axis, args.yaw_axis,
             args.throttle_axis, args.arm_button, args.deadman_button)):
        p.error("Supply --address, all four --*-axis mappings, --arm-button and --deadman-button")
    try:
        asyncio.run(pilot(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
