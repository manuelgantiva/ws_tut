from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld =LaunchDescription()

    remap_velocity_1= ("rpm", "rpm_1")
    remap_velocity_2= ("rpm", "rpm_2")

    encoder_car1_node = Node (
        package="my_ude_cpp",
        executable="encoder",
        name="encoder_1",
        remappings=[
            remap_velocity_1
        ],
        parameters=[
            {"rpm_value" : 10.0}
        ]
    )

    encoder_car2_node = Node (
        package="my_ude_py",
        executable="encoder.py",
        name="encoder_2",
        remappings=[
            remap_velocity_2
        ],
        parameters=[
            {"rpm_value" : 2.0}
        ]
    )

    speedometer_car1_node = Node (
        package="my_ude_cpp",
        executable="speedometer",
        name="speedometer_1",
        remappings=[
            remap_velocity_1
        ],
        parameters=[
            {"wheel_radio" : 0.5}
        ]
    )

    speedometer_car2_node = Node (
        package="my_ude_py",
        executable="speedometer.py",
        name="speedometer_2",
        remappings=[
            remap_velocity_2
        ],
        parameters=[
            {"wheel_radio" : 1.5}
        ]
    )

    show_topic = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )

    ld.add_action(encoder_car1_node)
    ld.add_action(encoder_car2_node)
    ld.add_action(speedometer_car1_node)
    ld.add_action(speedometer_car2_node)
    ld.add_action(show_topic)



    return ld