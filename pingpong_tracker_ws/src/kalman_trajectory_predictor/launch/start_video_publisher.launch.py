from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

#可以启动节点，但是参数加载失败，暂时先硬编码在cpp文件中，我吐了，为什么！之后来填

def generate_launch_description():
    pkg_name = 'kalman_trajectory_predictor'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare launch arguments
    image_source_arg = DeclareLaunchArgument(
        'image_source',
        default_value='video',
        description='Image source type (video or camera).'
    )

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=os.path.join(pkg_dir, 'test_video', 'pingpong_video.mp4'),
        description='Path to the video file to be published.'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file://' + os.path.join(pkg_dir, 'config', 'camera_info.yaml'),
        description='URL for the camera info YAML file.'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID for video capture.'
    )

    # 定义可组合节点
    nodes_to_compose = [
        ComposableNode(
            package=pkg_name,
            plugin=f'{pkg_name}::ImagePublisherNode',
            name='image_publisher_node',
            parameters=[
                {'image_source': LaunchConfiguration('image_source')},
                {'video_path': LaunchConfiguration('video_path')},
                {'camera_info_url': LaunchConfiguration('camera_info_url')},
                {'camera_id': LaunchConfiguration('camera_id')},
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    # 创建容器并运行
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package=pkg_name,
        executable=f'{pkg_name}_container_executable', 
        composable_node_descriptions=nodes_to_compose,
        output='screen'
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        image_source_arg,
        video_path_arg,
        camera_info_url_arg,
        camera_id_arg,
        container
    ])