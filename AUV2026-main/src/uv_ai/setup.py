from setuptools import setup
import os

package_name = 'uv_ai'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Path to datas relative to setup.py (src/uv_ai/setup.py)
    datas_base = os.path.join(os.path.dirname(__file__), '..', 'datas')
    
    if os.path.exists(datas_base):
        files = [os.path.join('..', 'datas', f) for f in os.listdir(datas_base) if os.path.isfile(os.path.join(datas_base, f))]
        data_files.append((os.path.join('share', package_name, 'datas'), files))
            
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(),
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
            "pc_recorder = uv_ai.pc_recorder:main",
            "uv_position = uv_ai.uv_position:main",
        ],
    },
)
