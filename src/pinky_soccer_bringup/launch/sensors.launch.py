"""Start robot sensor processes without starting any motor controller."""

from pinky_soccer_bringup.sensor_launch import generate_sensor_description


def generate_launch_description():
    """Return the shared camera and optional IMU/ADC process configuration."""
    return generate_sensor_description()
