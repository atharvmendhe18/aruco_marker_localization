from setuptools import find_packages, setup

package_name = 'markar_localization'

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
    maintainer='atharv',
    maintainer_email='atharvmendhe18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "calculate_intersection = markar_localization.calculate_position_based_on_2_markers:main",
        "imu_to_yaw = markar_localization.imu_to_yaw:main",
        "yaw_based_position = markar_localization.calculate_position_based_on_yaw:main",
        "aurco_marker_detection = markar_localization.aruco_marker_detection:main",
        ],
    },
)
