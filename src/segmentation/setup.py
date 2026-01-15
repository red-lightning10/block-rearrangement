from setuptools import setup

package_name = 'segmentation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/checkpoints', ['checkpoints/model_final.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niranjan Kumar Ilampooranan',
    maintainer_email='inkredible2599@gmail.com',
    description='Segmentation service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation_server = segmentation.segmentation_server:main',
            'trigger_segmentation = segmentation.trigger_segmentation:main',
        ],
    },
)

