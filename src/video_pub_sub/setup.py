from setuptools import find_packages, setup

package_name = 'video_pub_sub'

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
    maintainer='girish',
    maintainer_email='girish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'video_publisher = video_pub_sub.video_publisher:main',
                'video_subscriber = video_pub_sub.video_subscriber:main',
                'ros2_to_gstreamer = video_pub_sub.ros2_to_gstreamer:main',
                'camera_publisher = video_pub_sub.camera_publisher:main',
        ],
    },
)
