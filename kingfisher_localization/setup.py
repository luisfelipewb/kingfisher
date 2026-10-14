from glob import glob

from setuptools import find_packages, setup

package_name = 'kingfisher_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/sites', glob('config/sites/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Felipe Wolf Batista',
    maintainer_email='luis.batista@gatech.edu',
    description='Kingfisher localization: SBG driver config, odom, IMU and GPS on the VRX topics, and site frames',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sbg_localization = kingfisher_localization.sbg_localization:main',
            'geo_tf_publisher = kingfisher_localization.geo_tf_publisher:main',
        ],
    },
)
