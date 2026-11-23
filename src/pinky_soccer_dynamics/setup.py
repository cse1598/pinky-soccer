from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_soccer_dynamics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'CALIBRATION.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='Pinky Pro wheel dynamics, parameter loader and command adapter',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'show_params = pinky_soccer_dynamics.tools:show_params',
            'check_wheel_signs = pinky_soccer_dynamics.tools:check_wheel_signs',
            'identify_sim = pinky_soccer_dynamics.tools:identify_sim',
        ],
    },
)
