from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sequencer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niranjan Kumar Ilampooranan',
    maintainer_email='inkredible2599@gmail.com',
    description='SMACH sequencer for pick actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smach_viewer = sequencer.smach_viewer:main',
            'pick_sequencer = sequencer.pick_sequencer:main',
            'pick_all_sequencer = sequencer.pick_all_sequencer:main',
            'pick_and_place_hc = sequencer.pick_and_place_hc:main',
            'pick_and_place_nhc = sequencer.pick_and_place_nhc:main',
        ],
    },
)
