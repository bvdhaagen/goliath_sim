import os
from glob import glob
from setuptools import setup

package_name = 'goliath_doors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arham',
    maintainer_email='bsso.cowlar@gmail.com',
    description='Automatic proximity sliding doors.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_doors = goliath_doors.auto_doors:main',
        ],
    },
)
