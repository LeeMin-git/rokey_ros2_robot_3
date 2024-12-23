from setuptools import find_packages, setup

package_name = 'min_ros2'

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
    maintainer='min',
    maintainer_email='leemin6487@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'img_pub=min_ros2.img_pub:main',
        'img_sub=min_ros2.img_sub:main',
        'img_saver=min_ros2.ros2_cam_save:main',
        'detect_arco=min_ros2.detect_arcomarker:main',
        ],
    },
)
