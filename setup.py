import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fizzer',
    maintainer_email='laurenluksay@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'time_trial_node = controller_pkg.time_trial_node:main',
            'camera_selector_node = controller_pkg.camera_selector_node:main',
        ],
    },
)
