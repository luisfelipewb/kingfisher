from setuptools import find_packages, setup

package_name = 'kingfisher_sail'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kingfisher',
    maintainer_email='kingfisher@todo.todo',
    description='Sail command interface, calibration and wind following',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sail_calibration = kingfisher_sail.sail_calibration:main',
            'sail_controller = kingfisher_sail.sail_controller:main',
            'sail_follow_wind = kingfisher_sail.sail_follow_wind:main',
        ],
    },
)
