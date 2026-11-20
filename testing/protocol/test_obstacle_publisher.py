"""Check production C publisher bytes against the dashboard's decoder."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "dashboard"))
from mqtt_protocol import decode_telemetry


class FirmwareProtocolTests(unittest.TestCase):
    def test_actual_c_obstacle_publisher(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / 'obstacle_publisher')
            compiled = subprocess.run([
                'cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                '-ffunction-sections', '-fdata-sections', '-include', 'stddef.h', '-DPROHIBIT_DEF_SIZE_T',
                '-D_PICO_RP2040_', '-DCNF_SMP=1', '-DTM_MQTT=1',
                '-Iinclude', '-Iconfig', 'testing/protocol/obstacle_publisher_fixture.c',
                'app_program/subsystem1/s1_comm.c', 'app_program/subsystem1/s1_wire.c',
                '-Wl,--gc-sections', '-o', output,
            ], cwd=root, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            packet = subprocess.run([output], check=True, capture_output=True).stdout
        self.assertEqual(len(packet), 7)
        self.assertEqual(decode_telemetry('obstacle', packet), dict(
            nearest_mm=220, width_mm=None, left_mm=None, left_open=True,
            right_mm=None, angle_deg=90, saved_profile=True))


if __name__ == "__main__":
    unittest.main()
