from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Đường dẫn file URDF
    urdf_file = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    ])

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                urdf_file
            ])
        }]
    )

    

    # # Include Gazebo launch 
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare("gazebo_ros"),
    #             "launch",
    #             "gazebo.launch.py"
    #         ])
    #     ])
    # )
    # # Spawn robot in Gazebo
    # spawn_entity_node = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-topic", "robot_description", "-entity", "my_robot"],
    #     output="screen"
    # )

    return LaunchDescription([
        
        robot_state_publisher_node,
        # gazebo_launch,
        # spawn_entity_node
    ])
