from setuptools import setup

package_name = 'telloDroneRacing'

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
    maintainer='ut',
    maintainer_email='ut@todo.todo',
    description='Drone racing controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = telloDroneRacing.controller:main',
        ],
    },
)
