from setuptools import find_packages, setup

package_name = 'sora_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'adafruit-circuitpython-motorkit'],
    zip_safe=True,
    maintainer='wslsora',
    maintainer_email='robin.wyffels@student.hogent.be',
    description='Motor driver node for TB6612FNG I2C shields',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver_node = sora_motor_driver.motor_driver_node:main',
        ],
    },
)
