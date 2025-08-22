import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir=get_package_share_directory('kalman_trajectory_predictor')
    params_file=os.path.join(pkg_dir,'config','params.yaml')

    container=ComposableNodeContainer(
        name='predictor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='kalman_trajectory_predictor',
                plugin='kalman_trajectory_predictor::PredictorNode',
                name='predictor_node',
                parameters=[params_file]
            ),
            ComposableNode(
                package='kalman_trajectory_predictor',
                plugin='kalman_trajectory_predictor::ImageVisualizerNode',
                name='image_visualizer_node',
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])