from setuptools import setup

package_name = 'uv_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Team',
    maintainer_email='user@example.com',
    description='Bridge node between original uv_msgs and micro-ROS zit6_interfaces',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microros_bridge = uv_bridge.microros_bridge:main',
            'uv_hmu_no_serial = uv_bridge.uv_hmu_no_serial:main',
        ],
    },
)
