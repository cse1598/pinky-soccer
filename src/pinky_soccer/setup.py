from setuptools import find_packages, setup

package_name = 'pinky_soccer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/reactive_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='Pinky soccer control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_pinky = pinky_soccer.hello_node:main',
            'reactive_controller = pinky_soccer.reactive_controller:main',
        ],
    },
)
