from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cleaning_bot'

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

            'cleaning_bot_llm = cleaning_bot.cleaning_bot_llm:main',

            'holonomic_position_controller_service = cleaning_bot.holonomic_position_controller_service:main',

            'spawn_waste = cleaning_bot.spawn_waste:spawn_ground_plane',

        ],
    },
)
