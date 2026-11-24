"""Prevent duplicate ADC readers, fabricated charge and stale battery publication."""

import importlib.util
from pathlib import Path
import time

from launch import LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import Node as LaunchNode
from pinky_soccer_bringup.battery_state_relay import battery_values, BatteryStateRelay
from pinky_soccer_bringup.sensor_launch import generate_sensor_description
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


def flatten(entities):
    """Launch entities including those nested in GroupActions (robot_name namespace group)."""
    out = []
    for entity in entities:
        out.append(entity)
        if isinstance(entity, GroupAction):
            out += flatten(entity.get_sub_entities())
    return out


@pytest.mark.parametrize('voltage,percent', [(6.4, 0), (7.2, 50), (8.4, 100)])
def test_nominal_percentage_uses_the_same_voltage_sample(voltage, percent):
    """Match the existing pinkylib mapping without rereading hardware."""
    assert battery_values(voltage) == pytest.approx((voltage, percent))


def test_bad_voltage_does_not_become_zero_percent():
    """Unknown battery readings are withheld, never substituted with a real-looking value."""
    for voltage in (0., -1., float('nan'), float('inf')):
        with pytest.raises(ValueError):
            battery_values(voltage)


def test_battery_readers_are_mutually_exclusive(tmp_path, monkeypatch):
    """Exercise the actual launch conditions, including description-only mode."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    path = Path(__file__).parents[1]/'launch/robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch_battery_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = flatten(module.generate_launch_description().entities)
    official = next(a for a in actions if isinstance(a, LaunchNode)
                    and a.node_executable == 'battery_publisher')
    adapter = next(a for a in generate_sensor_description().entities if isinstance(a, LaunchNode)
                   and a.node_executable == 'battery_state_relay')
    for hardware in ('true', 'false'):
        for adc in ('true', 'false'):
            context = LaunchContext()
            context.launch_configurations.update(hardware=hardware, adc_enabled=adc)
            assert official.condition.evaluate(context) == (hardware == 'true' and adc == 'false')
            assert adapter.condition.evaluate(context) == (hardware == 'true' and adc == 'true')


def test_relay_requires_fresh_valid_upstream_and_preserves_voltage(tmp_path, monkeypatch):
    """Drive real DDS input/output; the adapter never opens an I2C device."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=94)
    relay = BatteryStateRelay()
    fixture = Node('battery_relay_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    executor.add_node(fixture)
    received = {'voltage': [], 'percent': []}
    fixture.create_subscription(Float32, 'battery/voltage',
                                lambda m: received['voltage'].append(m.data), 10)
    fixture.create_subscription(Float32, 'battery/percent',
                                lambda m: received['percent'].append(m.data), 10)
    upstream = fixture.create_publisher(BatteryState, 'batt_state', qos_profile_sensor_data)

    def pump(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    try:
        pump(.3)
        relay.publish()
        pump(.1)
        assert not received['voltage']
        deadline = time.monotonic()+3
        while relay.sample is None and time.monotonic() < deadline:
            upstream.publish(BatteryState(voltage=7.2, percentage=float('nan')))
            pump(.05)
        assert relay.sample is not None
        relay.publish()
        pump(.2)
        assert received['voltage'] == pytest.approx([7.2])
        assert received['percent'] == pytest.approx([50.], abs=.01)
        relay.sample = (time.monotonic()-3, relay.sample[1])
        relay.publish()
        pump(.1)
        assert len(received['voltage']) == 1
        relay.receive(BatteryState(voltage=float('nan')))
        assert relay.sample is None
        relay.publish()
        pump(.1)
        assert len(received['voltage']) == 1
    finally:
        executor.shutdown()
        relay.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
