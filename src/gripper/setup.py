from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'gripper'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name, f'{package_name}.utils'],
    #.so object is the linked library to our gripper's functionalities
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['lib/librh_p12_rna.so']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'std_srvs', 'pyyaml'],
    zip_safe=True,
    maintainer='Niranjan Kumar Ilampooranan',
    maintainer_email='inkredible2599@gmail.com',
    description='Gripper control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gripper_server = gripper.control_gripper_server:main',
            'gripper_action_server = gripper.gripper_action_server:main',
            'joint_state_merger = gripper.joint_state_merger:main',
            'gripper_test_client = gripper.gripper_test_client:main',
        ],
    },
)

