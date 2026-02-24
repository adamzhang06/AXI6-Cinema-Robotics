from setuptools import setup

package_name = 'axi6_camera_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 package for AXI6 Cinema Robotics system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = axi6_camera_system.vision_node:main',
            'controller_node = axi6_camera_system.controller_node:main',
            'hardware_node = axi6_camera_system.hardware_node:main',
        ],
    },
)
