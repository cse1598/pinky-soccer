from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_soccer_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'REAL_GAME.md',
                                   'GAMEPLAY.md', 'VERIFY.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='World model UIs and simulation launches for Pinky soccer',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'world_model_viz = pinky_soccer_viz.world_model_viz:main',
            'game_camera_view = pinky_soccer_viz.game_camera_view:main',
            'world_model_topview = pinky_soccer_viz.world_model_topview:main',
            'mujoco_eval_bridge = pinky_soccer_viz.mujoco_eval_bridge:main',
            'soccer_trial = pinky_soccer_viz.soccer_trial:main',
            'behavior_verifier = pinky_soccer_viz.behavior_verifier:main',
            'goto_bench = pinky_soccer_viz.goto_bench:main',
            'gazebo_truth_relay = pinky_soccer_viz.gazebo_truth_relay:main',
            'game_referee = pinky_soccer_viz.game_referee:main',
            'obstacle_driver = pinky_soccer_viz.obstacle_driver:main',
        ],
    },
)
