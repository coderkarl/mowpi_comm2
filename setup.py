from setuptools import setup

package_name = 'mowpi_comm2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karl',
    maintainer_email='21990134+coderkarl@users.noreply.github.com',
    description='Serial ROS bridge for mowpi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mowpi_comm = mowpi_comm2.mowpi_ros_comm:main",
            "gps_fusion = mowpi_comm2.mowpi_gps_fusion:main",
            "descent_comm = mowpi_comm2.descent_ros_comm:main",
        ],
    },
)
