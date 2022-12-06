# MIT License

# Copyright (c) 2022 Guru Nandhan A D P

# Permission is hereby granted, free of charge, to any person obtaining a 
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the 
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
# DEALINGS IN THE SOFTWARE.

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

##
# @brief this launch file record publisher data
#
##
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