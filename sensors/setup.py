from setuptools import setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='adonais.ray.maclang@eee.upd.edu.ph',
    description='Sensor nodes',
    license='Apached 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'subscriber = sensors.subscriber:main',
		'sensors = sensors.sensors:main',
		'timer_node = sensors.timer_node:main',
                'camera_publisher_node = sensors.camera_publisher_node:main',
                'camera_subscriber_node = sensors.camera_subscriber_node:main',
        ],
    },
)
