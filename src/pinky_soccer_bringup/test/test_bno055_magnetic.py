"""Check BNO055 register decoding against explicit known physical units."""

import math
import struct

from pinky_soccer_bringup.bno055_magnetic import BNO055, decode_sample
import pytest


def test_ndof_register_units_and_quaternion():
    data = struct.pack('<16h', 0, 0, 981, 352, -160, -672, 0, 0, 1440,
                       0, 0, 0, 16384, 0, 0, 0)
    accel, gyro, mag, q = decode_sample(data)
    assert accel == pytest.approx([0, 0, 9.81])
    assert gyro == pytest.approx([0, 0, math.pi/2])
    assert mag == pytest.approx([22e-6, -10e-6, -42e-6])
    assert q == [0, 0, 0, 1]
    with pytest.raises(ValueError):
        decode_sample(bytes(32))


def test_ndof_startup_uses_atomic_register_transfers_and_readback(monkeypatch):
    from pinky_soccer_bringup import bno055_magnetic as driver
    registers = {0: 0xA0}
    transfers, closed = [], []
    monkeypatch.setattr(driver.os, 'open', lambda *args: 71)
    monkeypatch.setattr(driver.os, 'close', closed.append)
    monkeypatch.setattr(driver.fcntl, 'flock', lambda *args: None)
    monkeypatch.setattr(driver.time, 'sleep', lambda value: None)

    def ioctl(fd, operation, request):
        assert fd == 71 and operation == 0x0707
        messages = [request.msgs[i] for i in range(request.nmsgs)]
        assert all(msg.addr == 0x28 for msg in messages)
        transfers.append([(m.flags, m.len) for m in messages])
        register = messages[0].buf[0]
        if len(messages) == 2:
            assert messages[0].len == 1 and messages[1].flags == 1
            for i in range(messages[1].len):
                messages[1].buf[i] = registers.get(register+i, 0)
        else:
            registers[register] = messages[0].buf[1]
        return 0

    monkeypatch.setattr(driver.fcntl, 'ioctl', ioctl)
    sensor = BNO055('/dev/test')
    assert registers[0x3D] == 0x0C and registers[0x3B] == 0
    assert [(0, 1), (1, 1)] in transfers
    sensor.close()
    assert closed == [71]
    registers[0] = 0
    with pytest.raises(OSError, match='chip ID'):
        BNO055('/dev/test')
    assert closed == [71, 71]
