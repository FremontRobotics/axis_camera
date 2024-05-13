from setuptools import find_packages, setup
import os 
from glob import glob
 
package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.4.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),

     ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Kelly',
    maintainer_email='3310379+JasonLKelly@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
             'axis = axis_camera.axis:main',
             'axis_ptz = axis_camera.axis_ptz:main',
             'axis_teleop = axis_camera.teleop:main',
             'axis_teleop_speed_control = axis_camera.teleop_speed_control:main',
             'publish_axis_tf = axis_camera.publish_axis_tf:main'
        ],
    },
)
