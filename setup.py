from setuptools import find_packages, setup

package_name = 'teleoperation_interface'

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
    maintainer='cdonoso',
    maintainer_email='clemente.donosok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = teleoperation_interface.camera_publisher:main',
            'display_information  = teleoperation_interface.display_information:main',
            'imu_node = teleoperation_interface.imu_publisher:main',
            'random_markers = teleoperation_interface.random_markers:main'
        ],
    },
)
