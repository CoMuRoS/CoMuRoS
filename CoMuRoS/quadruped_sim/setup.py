from setuptools import find_packages, setup

package_name = 'quadruped_sim'

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
    maintainer='bhavi',
    maintainer_email='bhavishraib@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_go2 = quadruped_sim.llm_go2:main',
            "file_pub = quadruped_sim.file_data_publisher:main",
        ],
    },
)
