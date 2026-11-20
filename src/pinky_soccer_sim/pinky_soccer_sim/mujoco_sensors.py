"""Extend official MuJoCo runtime classes without modifying the submodule."""

from pinky_soccer_sim.sensor_model import add_mujoco_sensors, load_config, RANGE_LINKS, ray_ranges
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def install(bridge, options):
    """Configure the renderer before model building and add physics-derived sensors."""
    cfg = load_config(options.sensor_config)
    original_build = bridge.build_model
    original_physics = bridge.Physics
    original_bridge = bridge.Bridge

    def build(*args, **kwargs):
        return add_mujoco_sensors(original_build(*args, **kwargs))

    class Physics(original_physics):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            # Camera geometry is configured in the generated physics YAML by launch.
            self.settings['camera']['topic'] = 'sim/camera/image_raw'
            self.settings['camera']['info_topic'] = 'sim/camera/camera_info'

    class Bridge(original_bridge):
        def __init__(self, physics):
            super().__init__(physics)
            self.soccer_imu = self.create_publisher(Imu, 'sim/imu', qos_profile_sensor_data)
            self.soccer_ranges = {
                name: self.create_publisher(LaserScan, f'sim/{name}/scan', qos_profile_sensor_data)
                for name in RANGE_LINKS}

        def advance(self):
            super().advance()
            now = bridge.stamp(self.physics.data.time)
            if options.imu_enabled:
                data = self.physics.data
                msg = Imu()
                msg.header.stamp, msg.header.frame_id = now, 'imu_link'
                w, x, y, z = map(float, data.sensor('soccer_quat').data)
                msg.orientation.w, msg.orientation.x = w, x
                msg.orientation.y, msg.orientation.z = y, z
                for field, name in [('angular_velocity', 'gyro'), ('linear_acceleration', 'acc')]:
                    values = map(float, data.sensor(f'soccer_{name}').data)
                    vector = getattr(msg, field)
                    vector.x, vector.y, vector.z = values
                # Zero covariance = unknown; this is an ideal sensor, not calibrated noise.
                self.soccer_imu.publish(msg)
            if options.adc_enabled and self.tick % 5 == 0:
                for name, publisher in self.soccer_ranges.items():
                    item = cfg['ultrasonic' if name == 'ultrasonic_link' else 'ir']
                    scan = LaserScan()
                    scan.header.stamp, scan.header.frame_id = now, name
                    scan.range_min, scan.range_max = item['min_range'], item['max_range']
                    scan.ranges = ray_ranges(self.physics, name, cfg)
                    publisher.publish(scan)

    bridge.build_model, bridge.Physics, bridge.Bridge = build, Physics, Bridge
