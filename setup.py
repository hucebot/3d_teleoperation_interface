from setuptools import find_packages, setup

package_name = 'ros2_3d_interface'

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
            '3d_interface = ros2_3d_interface.3d_interface:main',
            '3d_recorder = ros2_3d_interface.record_pcl:main',
            '3d_real_time = ros2_3d_interface.3d_real_time:main',
            'publish_trajectory = ros2_3d_interface.trajectory:main',
            'verify_trajectory = ros2_3d_interface.trajectory_verify:main'
        ],
    },
)
