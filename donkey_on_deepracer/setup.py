import os
from glob import glob
from setuptools import setup

package_name = 'donkey_on_deepracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haoru Xue',
    maintainer_email='hxue@ucsd.edu',
    description='Run Donkeycar on Deepracer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'donkey_interface_node = donkey_on_deepracer.donkey_interface_node:main'
        ],
    },
)
