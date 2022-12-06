from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    """Method to launch the nodes in the package with bag record flag"""
    bag_record = LaunchConfiguration('bag_record')

    bag_record_arg = DeclareLaunchArgument(
        'bag_record',
        default_value='False'
    )
    talker_node = Node(
        package='cpp_pubsub',
        executable='talker'
    )
    listener_node = Node(
        package='cpp_pubsub',
        executable='listener'
    )
    server_node = Node(
        package='cpp_pubsub',
        executable='server_client'
    )
    bag_record_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([bag_record,' == True'])
        ),
        cmd=[[
            'cd ../results/bag_files && ros2 bag record /chatter '
        ]],
        shell=True
    )

    return LaunchDescription([
        bag_record_arg,
        talker_node,
        listener_node,
        server_node,
        TimerAction(
            period=2.0,
            actions=[bag_record_conditioned],
        )
    ])