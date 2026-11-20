from setuptools import find_packages, setup

setup(
    name='pinky_soccer_calibration', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_soccer_calibration']),
        ('share/pinky_soccer_calibration', ['package.xml', 'README.md', 'WORKBENCH.md', 'ROS_UI.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    maintainer='Pinky Soccer maintainers', maintainer_email='maintainer@example.com',
    description='Hardware calibration experiment recording and validation',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_intrinsics = pinky_soccer_calibration.cli:main',
        'camera_workbench = pinky_soccer_calibration.workbench:main',
        'calibration_ui = pinky_soccer_calibration.suite_ui:main',
    ]},
)
