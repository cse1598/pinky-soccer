from setuptools import find_packages, setup

package_name = 'pinky_soccer_field'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse',
    maintainer_email='102153140+cse1598@users.noreply.github.com',
    description='RCJ soccer field geometry and LiDAR map generation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'generate_field_map = pinky_soccer_field.generate_field_map:main',
        ],
    },
)
