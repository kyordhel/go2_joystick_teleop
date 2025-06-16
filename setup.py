from setuptools import setup

package_name = 'joystick_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mauricio Matamoros',
    maintainer_email='kyordhel@gmail.com',
    description='Forward /joy to /cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = joystick_teleop.joystick_teleop:main',
        ],
    },
)
