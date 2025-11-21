from setuptools import setup

package_name = 'sbr_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 nodes for Self-Balancing Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_commander = sbr_nodes.vision_commander:main',
            'serial_bridge = sbr_nodes.serial_bridge:main',
            'follow_node = sbr_nodes.follow_node:main',
        ],
    },
)
