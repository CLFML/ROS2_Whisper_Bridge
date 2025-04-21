import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:
    # Declare launch arguments
    active_arg = DeclareLaunchArgument(
        'active',
        default_value="true",
        description='Start with whisper node active'
    )
    
    domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value="0",
        description='ROS domain ID for cross-machine communication'
    )
    
    # Get the substitution references
    active = LaunchConfiguration('active')
    domain_id = LaunchConfiguration('ros_domain_id')

    # Launch description
    ld = LaunchDescription()
    
    # Add command line arguments
    ld.add_action(active_arg)
    ld.add_action(domain_id_arg)

    # Launch the audio_bridge node to receive audio from the client
    ld.add_action(
        Node(
            package="whisper_bridge",
            executable="audio_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": "/audio_stamped",
                "output_topic": "/audio_listener/audio"
            }],
            # Set ROS_DOMAIN_ID through environment variables
            additional_env={'ROS_DOMAIN_ID': domain_id}
        )
    )

    # Find the whisper configuration file
    whisper_config = os.path.join(
        get_package_share_directory("whisper_server"), "config", "whisper.yaml"
    )

    # Launch the whisper container with inference and transcript_manager nodes
    container = ComposableNodeContainer(
        name='whisper_container',
        package='rclcpp_components',
        namespace='',
        executable='component_container_mt',  # Multi-threaded for better performance
        output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        emulate_tty=True,
        # Set ROS_DOMAIN_ID through environment variables
        additional_env={'ROS_DOMAIN_ID': domain_id},
        composable_node_descriptions=[
            # Whisper inference node
            ComposableNode(
                package='whisper_server',
                plugin='whisper::Inference',
                name='inference',
                namespace="whisper",
                parameters=[whisper_config, {'active': active}],
                remappings=[("audio", "/audio_listener/audio")],
            ),
            # Transcript manager
            ComposableNode(
                package='transcript_manager',
                plugin='whisper::TranscriptManager',
                name='transcript_manager',
                namespace="whisper",
            ),
        ],
    )
    
    ld.add_action(container)
    return ld