from setuptools import setup

package_name = 'capstone_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hle',
    maintainer_email='huyftw@aim.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "basic_movement = capstone_pkg.basic_movement:main",
            "stop_movement = capstone_pkg.stop_movement:main",
            "tb3_status = capstone_pkg.tb3_status:main",
            "circle_around = capstone_pkg.circle_around:main",
            "occupancy_grid = capstone_pkg.occupancy_grid:main",
            "robot_world = capstone_pkg.robot_world:main",
            "transforms = capstone_pkg.transforms:main",
            "laser_geomtry = laser_geometry.src.laser_geometry"
        ],
    },
)
