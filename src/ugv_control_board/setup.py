from setuptools import find_packages, setup

package_name = 'ugv_control_board'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ugv_serial_control_launch.py']), 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_processor = ugv_control_board.serial_processor_node:main",
            "velocity_command = ugv_control_board.velocity_command_node:main",
            "msg_out = ugv_control_board.msg_out_node:main" 
 
        ],
    },
)
