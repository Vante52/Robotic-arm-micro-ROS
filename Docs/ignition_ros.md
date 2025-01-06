## IGNITION <-> ROS2

Create a Launch File that:
1. Set the package directory
```python
pkg_robot_gz = get_package_share_directory('diffbot_description')
```
2. Set the simulation environment
```python
rsp_arg = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.')
ros_bridge_arg = DeclareLaunchArgument('ros_bridge', default_value='true', description='Start ROS bridge.')

world_name = 'ras_world.sdf'
world_path = os.path.join(pkg_robot_gz, 'worlds', world_name)
```
3. Create a robot Description
```python 
def create_robot_description(context):
    xacro_file = os.path.join(pkg_robot_gz, 'urdf', 'robot.urdf.xacro')
    

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    robot_desc = robot_desc.replace(
    'package://diffbot_description/', f'file://{pkg_robot_gz}/'
    )

    # Set the robot_description launch configuration
    return [SetLaunchConfiguration('robot_description', robot_desc)]

create_robot_description_fn = OpaqueFunction(function=create_robot_description)
params = {'robot_description': LaunchConfiguration('robot_description'),
            'publish_frequency': 30.0,
            'use_sim_time': True}
```
4. Set the robot state publisher
```python 
# Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )
```
5. Init an Ignition Gazebo Simulation
```python 
# Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )
```
6. Spawn the robot Model
```python 
# Spawn the robot model.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "diffbot",
            '-topic', "/robot_description",
            '-x', '0',
            '-y', '0',
            '-z', '1.0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ],
        output='screen',
    )
```
7. Create a bridge between the gazebo and ROS.
```python 
# ROS bridge
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
```

So the resulting launch file will be:

```python 
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

pkg_robot_gz = get_package_share_directory('diffbot_description')

def generate_launch_description():

    rsp_arg = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.')
    ros_bridge_arg = DeclareLaunchArgument('ros_bridge', default_value='true', description='Start ROS bridge.')

    world_name = 'ras_world.sdf'
    world_path = os.path.join(pkg_robot_gz, 'worlds', world_name)

    def create_robot_description(context):
        xacro_file = os.path.join(pkg_robot_gz, 'urdf', 'robot.urdf.xacro')
        

        robot_description_config = xacro.process_file(xacro_file)
        robot_desc = robot_description_config.toxml()
        robot_desc = robot_desc.replace(
        'package://diffbot_description/', f'file://{pkg_robot_gz}/'
        )

        # Set the robot_description launch configuration
        return [SetLaunchConfiguration('robot_description', robot_desc)]

    create_robot_description_fn = OpaqueFunction(function=create_robot_description)
    params = {'robot_description': LaunchConfiguration('robot_description'),
              'publish_frequency': 30.0,
              'use_sim_time': True}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    # Spawn the robot model.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "diffbot",
            '-topic', "/robot_description",
            '-x', '0',
            '-y', '0',
            '-z', '1.0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ],
        output='screen',
    )

    # ROS bridge
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription(
        [
            ros_bridge_arg,
            rsp_arg,
            rviz_arg,
            create_robot_description_fn,
            gazebo,
            rsp,
            spawn_node,
            ros_bridge
        ]
    )
```

## Next
-  [Activity](Activity.md)
## Previous
-  [Ignition Plugin](ignition_plugin.md)