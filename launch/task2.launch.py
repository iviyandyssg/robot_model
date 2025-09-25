import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # 载入不同的世界和机器人需要修改的参数
    urdf_name = 'robot.urdf'

    rviz_name = 'test_robot_rviz2.rviz'

    #world_name = 'large_annulus.world'
    #world_name = 'large_playground.world'
    #world_name = 'large_rectangle.world'
    #world_name = 'medium_annulus.world'
    #world_name = 'medium_playground.world'
    #world_name = 'medimu_rectangle.world'
    #world_name = 'small_annulus.world'
    #world_name = 'small_playground.world'
    #world_name = 'small_rectangle.world'
    world_name = 'grotesque.world'
    
    robot_name_in_model = 'test_robot'
    pkg_name = 'robot_model'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    gazebo_world_path = os.path.join(pkg_share, f'world/{world_name}')
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    rviz_config_path = os.path.join(pkg_share, f'rviz/{rviz_name}')

    # 启动gazebo并载入环境数据
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen'
    )

    # 载入机器人到gazebo世界中
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model, 
            '-file', urdf_model_path,
            '-x', '0.0',  # 原点X坐标
            '-y', '0.0',  # 原点Y坐标
            '-R', '0.0',  # Roll角（绕X轴旋转）
            '-P', '0.0',  # Pitch角（绕Y轴旋转）
            '-Y', '0.0'   # Yaw角（绕Z轴旋转，0表示正方向）
            ],
        output='screen'
    )

    # 广播机器人模型
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    # 启动rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 添加noise
    odom_noise_cmd = Node(
        package='robot_model',
        executable='odom_node',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(odom_noise_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    
    return ld

