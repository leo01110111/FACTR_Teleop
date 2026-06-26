from setuptools import find_packages, setup

package_name = 'factr_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Jingzhou Liu and Yulong Li',
    maintainer_email='liujason@cmu.edu',
    description='FACTR low-cost force-feedback teleoperation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'factr_teleop_franka = factr_teleop.factr_teleop_franka_zmq:main',
            'factr_teleop_ur7e = factr_teleop.factr_teleop_ur7e:main',
            'ur7e_collision_monitor = factr_teleop.ur7e_collision_monitor:main',
            'isaac_rmpflow_stream_bridge = factr_teleop.isaac_rmpflow_stream_bridge:main',
            'factr_teleop_grav_comp_demo = factr_teleop.factr_teleop_grav_comp_demo:main',
            'return_ur_to_initial_match = factr_teleop.return_ur_to_initial_match:main',
        ],
    },
)
