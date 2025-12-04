import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions
def generate_launch_description():
    #获取功能包的share路径
    urdf_package_path = get_package_share_directory('cleanbot_description') 
    default_xacro_path = os.path.join(urdf_package_path, 'urdf','cleanbot', 'clean_robot.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    #声明一个urdf参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model',default_value = str(default_xacro_path),description = '加载模型文件路径'
    )

    #通过路径，获取内容，转换为参数值对象，传入robot_state_publisher节点
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)


    action_robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
        )
    
    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package = 'joint_state_publisher',
    #     executable='joint_state_publisher',
    # )

    # action_rivz_node = launch_ros.actions.Node(
    #     package = 'rviz2',
    #     executable='rviz2',
    #     arguments=['-d',default_rviz_config_path]
    # )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true') ]
    
    )

    action_spwan_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description', '-entity','cleanbot'],
        output='screen'
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'cleanbot_joint_state_broadcaster','--set-state', 'active'],
        output='screen'
    )

    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'clean_effort_controller','--set-state', 'active'],
        output='screen'
    )

    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'clean_diff_drive_controller','--set-state', 'active'],
        output='screen'
    )    

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spwan_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spwan_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        )
        # action_rivz_node
        ])