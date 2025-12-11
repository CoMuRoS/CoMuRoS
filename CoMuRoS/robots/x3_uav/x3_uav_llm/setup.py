from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'x3_uav_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='name',
    maintainer_email='name@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'x3_uav_llm = x3_uav_llm.x3_uav_llm:main',

            'drone_position_controller_service = x3_uav_llm.drone_position_controller_service:main',
            'drone_position_controller_client = x3_uav_llm.drone_position_controller_client:main',
        ],
    },
)
