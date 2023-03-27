from setuptools import setup

package_name = 'sensor_topics'

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
    maintainer='adonais.ray.maclang',
    maintainer_email='adonais.ray.maclang@eee.upd.edu.ph',
    description='Sensor and timer nodes',
    license='Apached 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'subscriber = sensors.subscriber:main',
		'sensors = sensor_topics.sensors:main',
		'periodic = sensor_topics.periodic:main',
        ],
    },
)
