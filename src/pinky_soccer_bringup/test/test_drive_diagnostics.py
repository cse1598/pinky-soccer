"""drive_diagnostics decodes the wider register block like the official driver (no hardware)."""
import importlib
import struct
import sys
import types


class FakeBulk:
    def __init__(self, port, packet, table):
        self.table, self.params = table, []

    def clearParam(self):
        self.params = []

    def addParam(self, dxl_id, addr, length):
        self.params.append((dxl_id, addr, length))
        return True

    def txRxPacket(self):
        return 0

    def isAvailable(self, dxl_id, addr, length):
        return True

    def getData(self, dxl_id, addr, length):
        return int.from_bytes(self.table[dxl_id][addr:addr + length], 'little')


def load(monkeypatch, table):
    sdk = types.ModuleType('dynamixel_sdk')
    sdk.COMM_SUCCESS = 0
    sdk.GroupBulkRead = lambda port, packet: FakeBulk(port, packet, table)
    monkeypatch.setitem(sys.modules, 'dynamixel_sdk', sdk)
    bringup = types.ModuleType('pinky_bringup.bringup')

    class OfficialDriver:
        RPM_TO_VALUE_SCALE = 1 / 0.229
        ADDR_PROFILE_ACCEL = 108

        def __init__(self, port, baud, ids):
            self.DXL_IDS, self.portHandler, self.packetHandler = ids, None, None

        def get_feedback(self):
            return 'official-fallback'
    bringup.DynamixelDriver = OfficialDriver
    bringup.Pinky = object
    pkg = types.ModuleType('pinky_bringup')
    pkg.bringup = bringup
    monkeypatch.setitem(sys.modules, 'pinky_bringup', pkg)
    monkeypatch.setitem(sys.modules, 'pinky_bringup.bringup', bringup)
    sys.modules.pop('pinky_soccer_bringup.drive_diagnostics', None)
    return importlib.import_module('pinky_soccer_bringup.drive_diagnostics')


def block(pwm, current, vel, pos, vtraj, ptraj, volt):
    raw = bytearray(200)
    raw[124:146] = struct.pack('<hhiiiih', pwm, current, vel, pos, vtraj, ptraj, volt)
    return raw


def test_decode_matches_official_semantics(monkeypatch):
    table = {1: block(120, -15, 44, 5000, 50, 4990, 74),
             2: block(-120, 16, -44, -5000, -50, -4990, 74)}
    module = load(monkeypatch, table)
    driver = module.DiagnosticDriver('/dev/null', 1000000, [1, 2])
    rpm_l, rpm_r, pos_l, pos_r = driver.get_feedback()
    assert abs(rpm_l - 44 * .229) < 1e-9 and abs(rpm_r + 44 * .229) < 1e-9   # raw signs kept
    assert (pos_l, pos_r) == (5000, -5000)
    left, right = driver.last_diag['motors']
    assert left['velocity_trajectory'] == 50 and right['present_current'] == 16
    assert left['present_input_voltage'] == 74 and right['present_pwm'] == -120
    assert module.signed(0xFFFFFFFF, 4) == -1 and module.signed(0x7FFF, 2) == 0x7FFF
