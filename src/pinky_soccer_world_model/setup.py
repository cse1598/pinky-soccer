from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_soccer_world_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LOCALIZATION.md',
                                  'STARTUP_LOCALIZATION.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='Ball detection and world model for Pinky soccer',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ball_detector = pinky_soccer_world_model.ball_detector:main',
            'world_model_node = pinky_soccer_world_model.world_model_node:main',
            'goal_heading = pinky_soccer_world_model.goal_heading_node:main',
            'rectangle_localization = pinky_soccer_world_model.rectangle_localization_node:main',
            'fake_world_model = pinky_soccer_world_model.fake_world_model:main',
        ],
    },
)
