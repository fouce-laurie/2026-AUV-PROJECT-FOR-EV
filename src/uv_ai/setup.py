from setuptools import setup

package_name = 'uv_ai'

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
            "uv_detect = uv_ai.uv_detect:main",
            "uv_detect_test = uv_ai.uv_detect_test:main",
            "uv_automation = uv_ai.uv_automaton:main",
            "uv_detect_demo = uv_ai.uv_detect_demo:main",
            "uv_segment = uv_ai.uv_segment:main",
        ],
    },
)
