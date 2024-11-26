from setuptools import setup,find_packages
import os
from glob import glob
package_name = 'xrower_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deltax',
    maintainer_email='viminhtu.101002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher = xrower_gps.gps_publisher:main',
            'gps_subscriber = xrower_gps.gps_subscriber:main',
        ],
    },
)
