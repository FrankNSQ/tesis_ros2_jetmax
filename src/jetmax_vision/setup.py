from setuptools import find_packages, setup

package_name = 'jetmax_vision'

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
    maintainer='fsuser',
    maintainer_email='fsuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = jetmax_vision.color_detector:main',
            'color_sorter = jetmax_vision.color_sorter:main',
            'cube_state_publisher = jetmax_vision.cube_state_publisher:main',
            'calibrate_points = jetmax_vision.calibrate_points:main',
            'gazebo_cube_spawner = jetmax_vision.gazebo_cube_spawner:main',
            'gazebo_sort_executor = jetmax_vision.gazebo_sort_executor:main',
            'gazebo_state_keeper = jetmax_vision.gazebo_state_keeper:main',
        ],
    },
)