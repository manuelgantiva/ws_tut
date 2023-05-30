from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Retrieving path information 
from ament_index_python.packages import get_package_share_directory
from pathlib import Path 

# Utilizing launch files from other packages
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Working with environment variables
from launch.actions import SetEnvironmentVariable

# Simulation event handling 
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# Path Variables 
ignition_ros_package_path = get_package_share_directory("ros_gz_sim")
udemy_ros2_pkg_path = get_package_share_directory("sim_tut")
simulation_world_file_path = Path(udemy_ros2_pkg_path, "worlds/Wheeled_model_world.sdf").as_posix()
simulation_models_path = Path(udemy_ros2_pkg_path, "models").as_posix()



def generate_launch_description():
    ld =LaunchDescription()

    set_path= SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=simulation_models_path
    )
    
    simulation = ExecuteProcess(
        cmd=['gz', 'sim', '-r', simulation_world_file_path],
        output='screen'
    )

    bridge_vel=Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/wheeled_model/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"], 
        remappings=[("/model/wheeled_model/cmd_vel", "/cmd_vel")], # Remapping topic in launch file
        output="screen"
    )   

    Res=RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=simulation,
        on_exit=[EmitEvent(event=Shutdown())])
    )

    ld.add_action(set_path)
    ld.add_action(simulation)
    ld.add_action(bridge_vel)
    ld.add_action(Res)

    return ld