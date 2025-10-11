from setuptools import setup

package_name = 'posha_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/simple_workspace.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/' + package_name + '/config', ['config/simulation.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Posha Robotics Assignment Simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'advanced_planner = posha_simulation.advanced_planner:main',
            'simple_test = posha_simulation.simple_test:main',
        ],
    },
)