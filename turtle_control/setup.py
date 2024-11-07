from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='tarun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "p_control=turtle_control.P_controller:main",
        "pi_control=turtle_control.PI_controller:main",
        "pid_control=turtle_control.PID_controller:main"

        ],
    },
)
