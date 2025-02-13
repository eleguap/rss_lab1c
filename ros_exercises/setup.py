from setuptools import setup

package_name = 'ros_exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_first_launch.launch.xml', 'launch/static_tf_publisher.launch.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='racecar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros_exercises.simple_publisher:main',
            'listener = ros_exercises.simple_subscriber:main',
            'comptalk = ros_exercises.fake_scan_publisher:main',
            'complist = ros_exercises.open_space_publisher:main',
            'dynamic = ros_exercises.dynamic_tf_cam_publisher:main',
            'static = ros_exercises.static_tf_cam_publisher:main',
            'base2 = ros_exercises.base_link_tf_pub:main',
        ],
    },
)
