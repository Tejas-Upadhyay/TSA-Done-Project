import importlib.util
from pathlib import Path
import struct
import unittest

path = Path(__file__).resolve().parents[1] / "tools" / "ble_bench.py"
spec = importlib.util.spec_from_file_location("bench", path)
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


class Packets(unittest.TestCase):
    def test_packet(self):
        value = bench.packet(1)
        self.assertEqual(len(value), 16)
        self.assertEqual(value[:14].hex(), "a10001000000000000000000e803")
        self.assertEqual(value[14:], bytes.fromhex("b7b2"))

    def test_wrap_and_signed_axes(self):
        values = struct.unpack("<BBIhhhHH", bench.packet(2**32, -1000, 1000, -50, 2000))
        self.assertEqual(values[2:7], (0, -1000, 1000, -50, 2000))

    def test_invalid(self):
        for args in ({"throttle": 999}, {"roll": 1001}, {"flags": 16},
                     {"flags": bench.TEST, "roll": 0},
                     {"flags": bench.TEST|bench.ARM, "roll": 1},
                     {"flags": bench.TEST, "roll": 4, "throttle": 1151}):
            with self.assertRaises(ValueError):
                bench.packet(1, **args)

    def test_motor(self):
        self.assertEqual(len(bench.packet(5, roll=4, throttle=1150, flags=bench.TEST)), 16)


if __name__ == "__main__":
    unittest.main()
