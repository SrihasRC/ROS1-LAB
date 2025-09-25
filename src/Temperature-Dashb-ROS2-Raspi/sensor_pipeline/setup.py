from setuptools import find_packages, setup

package_name = 'sensor_pipeline'

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
    maintainer='srihasrc',
    maintainer_email='srihasrc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'sensor_bridge = sensor_pipeline.sensor_bridge:main',
        'sensor_compute = sensor_pipeline.sensor_compute:main',
        'sensor_cui = sensor_pipeline.sensor_cui:main',
        ],
    },
)
