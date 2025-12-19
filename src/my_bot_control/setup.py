from setuptools import setup

package_name = 'my_bot_control'

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
    maintainer='iram',
    maintainer_email='mukriiram10@gmail.com',
    description='Control and teleop for my robot flaps',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flap_teleop_gz = my_bot_control.flap_teleop_gz:main',
        ],
    },
)
