from setuptools import setup

package_name = 'uv_hm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macabaka',
    maintainer_email='macabaka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uv_hmu = uv_hm.uv_hmu:main",
            "uv_web_pannel = uv_hm.uv_web_pannel:main",
        ],
    },
)
