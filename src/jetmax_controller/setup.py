from setuptools import find_packages, setup

package_name = 'jetmax_controller'

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
            'arm_bridge_sim = jetmax_controller.arm_bridge_sim:main',
            'suction_attach_sim = jetmax_controller.suction_attach_sim:main',
        ],
    },
)

