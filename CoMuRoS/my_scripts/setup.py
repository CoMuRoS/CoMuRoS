from setuptools import find_packages, setup

package_name = 'my_scripts'

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
    maintainer='aarav',
    maintainer_email='aarav.shah@iitgn.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['initial_pose_publisher = my_scripts.initial_pose_publisher:main',  # Entry point for your node
            'turtlebot_nav = my_scripts.turtlebot_nav:main',
              'turtlebot3_pose_publisher = my_scripts.current_pose_publisher:main',  # Entry point for your other node
        ],
    },
)
