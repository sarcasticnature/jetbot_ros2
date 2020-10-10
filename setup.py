from setuptools import setup

package_name = 'jetbot_ros2'

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
    maintainer='Jake Keller',
    maintainer_email='jakeller3630@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = jetbot_ros2.motors:main',
            'distance_node = jetbot_ros2.vl53l1x:main',
        ],
    },
)
