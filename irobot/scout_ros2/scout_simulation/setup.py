from setuptools import setup

package_name = 'scout_simulation'

setup(
    name=package_name,
    version='1.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',  'launch/simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cong-Thanh Vu',
    maintainer_email='vuthanh.cdt@gmail.com',
    description='ROS-2 Simulation packages for Genbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
