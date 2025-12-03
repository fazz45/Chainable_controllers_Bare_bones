from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # paths to  configs
    hardware_config = "/home/fazil/ros/my_ws/src/chainable_controllers_demo/config/fake_hardware.yaml"
    controllers_config = "/home/fazil/ros/my_ws/src/chainable_controllers_demo/config/controllers.yaml"

    # Path to the URDF 
    urdf_file = "/home/fazil/ros/my_ws/src/chainable_controllers_demo/description/mock_chainable.urdf"

    # Load URDF 
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # robot_state_publisher, robot_description as a parameter
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    #  ros2_control_node also gets robot_description as a parameter
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            hardware_config,
            controllers_config,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # Spawner for  Kalman
    kalman_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kalman_controller"],
        output="screen",
    )


   
    broyden_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["broyden_controller"],
        output="screen",
    )

    rigid_pose_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rigid_pose_broadcaster"],
        output="screen",
    )

    # when kalman_spawner exits, start broyden and rigid
    start_mid_and_upstream = RegisterEventHandler(
        OnProcessExit(
            target_action=kalman_spawner,
            on_exit=[
                broyden_spawner,
                rigid_pose_spawner,
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager_node,
        kalman_spawner,          # starts first
        start_mid_and_upstream,  # triggers next 2
    ])
