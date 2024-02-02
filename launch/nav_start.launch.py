import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    # include xml launch file
   axis_ptz = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('axis_camera'), 'launch'),
            '/axis_ptz.launch']),
            )
   localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('clearpath_nav2_demos'), 'launch'),
            '/localization.launch.py']),
        launch_arguments={'setup_path': '/home/administrator/clearpath/', 'map': '/home/administrator/Officemapjackal2.yaml'}.items(),
    )    

   nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('clearpath_nav2_demos'), 'launch'),
            '/nav2.launch.py']),
        launch_arguments={'setup_path': '/home/administrator/clearpath/'}.items(),
    )
#   config = os.path.join(
#            get_package_share_directory('clearpath_nav2_demos'),
#            '/home/administrator/opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml'
#    )
   return LaunchDescription([
        axis_ptz,
        localization,
        nav2
    ])
