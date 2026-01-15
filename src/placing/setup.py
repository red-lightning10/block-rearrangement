from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'placing'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'hdbscan', 'opencv-python'],
    zip_safe=True,
    maintainer='Jeronimo Ruiz Fernandez',
    maintainer_email='jruiz1@wpi.edu',
    description='Placing service - handles object placement operations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'placement_server = placing.placement_server:main',
            'placement_client = placing.placement_client:main',
        ],
    },
)

