from setuptools import find_packages, setup

package_name = 'smart_trash_sensors'

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
    maintainer='carrotion',
    maintainer_email='59335136+carrotion@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_publisher_node = smart_trash_sensors.imu_publisher_node:main',
            'scan_filter_node = smart_trash_sensors.scan_filter_node:main',
        ],
    },
)
