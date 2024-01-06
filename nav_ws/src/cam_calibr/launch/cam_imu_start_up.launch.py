
from launch import LaunchDescription
import launch
import launch_ros.actions

from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name',         'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',    'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'config_file',         'default': [ThisLaunchFileDir(), "/config/config.yaml"], 'description': 'yaml config file'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])



def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
        launch_ros.actions.Node(
            package='imu-listener',
            executable='imu-listener',
            name='imu_node'),
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='cam_node'),
        OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params' : set_configurable_parameters(params)}
        )
  ])
