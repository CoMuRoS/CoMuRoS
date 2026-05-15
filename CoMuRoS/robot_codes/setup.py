from setuptools import find_packages, setup

package_name = 'robot_codes'

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
    maintainer='vipul',
    maintainer_email='vipul.pardeshi@flytbase.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ur5_commander=robot_codes.ur5_commander:main",
            "llm_ur5=robot_codes.llm_ur5:main",
            "file1=robot_codes.file_data_publisher1:main",
            "file2=robot_codes.file_data_publisher2:main",            

        ],
    },
)
