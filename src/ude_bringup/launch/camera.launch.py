from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld =LaunchDescription()

    remap_camera_1 = ("turn_camera", "turn_camera_1")
    remap_camera_2 = ("turn_camera", "turn_camera_2")

    turn_camera_server_1 = Node (
        package="my_ude_cpp",
        executable="turn_camera_server",
        name="Camera_1",
        remappings=[
            remap_camera_1
        ],
        parameters=[
            {"angle_initial" : 15.0},
            {"publish_frecuency" : 3.0}
        ]
    )

    turn_camera_client_1 = Node (
        package="my_ude_cpp",
        executable="turn_camera_client",
        name="Control_1",
        remappings=[
            remap_camera_1
        ],
        parameters=[
            {"client_time" : 15.0},
        ]
    )

    turn_camera_server_2 = Node (
        package="my_ude_py",
        executable="turn_camera_server.py",
        name="Camera_2",
        remappings=[
            remap_camera_2
        ],
        parameters=[
            {"angle_initial" : -15.0},
            {"publish_frecuency" : 4.0}
        ]
    )

    turn_camera_client_2 = Node (
        package="my_ude_py",
        executable="turn_camera_client.py",
        name="Control_2",
        remappings=[
            remap_camera_2
        ],
        parameters=[
            {"client_time" : 11.0},
        ]
    )


    show_service = ExecuteProcess(
        cmd=['ros2', 'service', 'list'],
        output='screen'
    )

    show_topic = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )

    ld.add_action(turn_camera_server_1)
    ld.add_action(turn_camera_client_1)
    ld.add_action(turn_camera_server_2)
    ld.add_action(turn_camera_client_2)
    ld.add_action(show_service)
    ld.add_action(show_topic)



    return ld