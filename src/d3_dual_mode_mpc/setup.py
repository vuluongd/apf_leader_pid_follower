from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'd3_dual_mode_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunog',
    maintainer_email='lunog@todo.todo',
    description='Dual-Mode MPC for Visual L-F UAV Formation Under Signal Loss',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_leader = d3_dual_mode_mpc.apf_leader_node:main',
            'signal_simulator = d3_dual_mode_mpc.signal_simulator_node:main',
            'mpc_follower = d3_dual_mode_mpc.mpc_follower_node:main',
            'velocity_relay = d3_dual_mode_mpc.velocity_relay_node:main',
            'data_logger = d3_dual_mode_mpc.data_logger_node:main',
        ],
    },
)
