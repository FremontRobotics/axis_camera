from setuptools import find_packages, setup
import os 
from glob import glob
 
package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.4.3',
    packages=['nodes'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Kelly',
    maintainer_email='jasonk@fremont.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
             'axis = nodes.axis:main',
             'axis_ptz = nodes.axis_ptz:main',
             'axis_teleop = nodes.teleop:main',
             'publish_axis_tf = nodes.publish_axis_tf:main'
        ],
    },
)
