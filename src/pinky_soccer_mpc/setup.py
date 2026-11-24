from setuptools import find_packages, setup

package_name = 'pinky_soccer_mpc'

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
    description='Pinky Pro NMPC (acados)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'check_acados = pinky_soccer_mpc.verify:main',
            'mpc_controller = pinky_soccer_mpc.mpc_controller:main',
        ],
    },
)
