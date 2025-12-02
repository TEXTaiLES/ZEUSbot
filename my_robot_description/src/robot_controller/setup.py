from setuptools import find_packages, setup

package_name = 'robot_controller'

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
    maintainer='hercules',
    maintainer_email='her.gousis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "go=robot_controller.go:main",
            "go_with_laser=robot_controller.go_with_laser:main",
            "go_with_lidar=robot_controller.go_with_lidar:main",
            "qr=robot_controller.qr:main",
            "marker_with_axis_rviz=robot_controller.marker_with_axis_rviz:main",
            "aruco_kalo=robot_controller.aruco_kalo:main",
            "ca=robot_controller.ca:main",
            "ransac=robot_controller.ransac:main",
            "go_dokari=robot_controller.go_dokari:main",
            "go_with_arm=robot_controller.go_with_arm:main",
            "position_detect=robot_controller.position_detect:main"
            
        ],
    },
)
