from setuptools import setup

package_name = 'jetmax_examples'

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
    maintainer='fsuser',
    maintainer_email='fsuser@todo.todo',
    description='Ejemplos de movimiento del robot JetMax',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploracion = jetmax_examples.exploracion:main',
            'saludo = jetmax_examples.saludo:main',
            'pick_and_place = jetmax_examples.pick_and_place:main',
            'path_visualizer = jetmax_examples.path_visualizer:main',
            'distance_marker = jetmax_examples.distance_marker:main',
        ],
    },
)