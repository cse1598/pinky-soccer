from glob import glob

from setuptools import find_packages, setup

package_name = 'pinky_soccer_skills'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'SKILLS.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
        ('share/' + package_name + '/docs/img', glob('docs/img/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='Pinky soccer skills, playbook, skill_player node and MuJoCo skill bench',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'skill_player = pinky_soccer_skills.skill_player:main',
            'skill_bench = pinky_soccer_skills.sim.bench:main',
        ],
    },
)
