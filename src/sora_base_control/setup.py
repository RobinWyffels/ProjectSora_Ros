from setuptools import setup

package_name = 'sora_base_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # THIS LINE IS REQUIRED
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),

        # And this if you have config files
        ('share/' + package_name + '/config', ['config/teleop_joy.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robin',
    maintainer_email='your@email',
    description='Teleop + mecanum control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mecanum_kinematics = sora_base_control.mecanum_kinematics:main',
        ],
    },
)
