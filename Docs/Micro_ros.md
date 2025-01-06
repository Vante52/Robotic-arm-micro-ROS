## micro-ROS Setup

1. Disabled the ros setup in the .bashrc file:
```sh
    #source ~/ros2_humble/install/setup.bash
```

2. Create a new Project with Platformio and VScode.
   
3. Go to platformio.ini and include at the end the following:
    1. micro-ROS repo
    ```ini
        ...
        lib_deps =
        https://github.com/micro-ROS/micro_ros_platformio

    ```
    2. ROS 2 distribution, in this example the ros distro is humble
    ```ini
        ...
        board_microros_distro = humble
    ```
    3. Transport configuration, the selection are between serial and wifi
       1. Serial:
           1. platformio.ini
            ```ini
                ...
                board_microros_transport = Serial
            ```
           2. main.cpp:
            ```cpp
                void setup() {
                ...
                set_microros_serial_transports(Serial);
                ...
                }
            ```
       2. Wifi:
           1. platformio.ini
            ```ini
                ...
                board_microros_transport = Wifi
            ```
           2. main.cpp:
            ```cpp
                void setup() {
                ...
                IPAddress agent_ip(127, 0, 0, 0); // PC Ip address
                size_t port = 8888;
                char ssid[] = "USERNAME";
                char psk[] = "PASSWORD";
                set_microros_wifi_transports(ssid, psk, agent_ip, port);
                delay(2000);
                ...
                }
            ```
4. Include the code that are on [robot_platformio](https://gitlab.com/gabrieldiaz94/ignition-tutorial/-/tree/main/robot_platformio/src?ref_type=heads) (or just upload the robot_platformio folder)
5. Enabled the ros setup in the .bashrc file:
```sh
    source ~/ros2_humble/install/setup.bash
```
1. Create a micro-ROS workspace:
```sh
    mkdir -p ~/microros_ws/src
    cd ~/microros_ws
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
1. Update the Dependencies
```sh
    sudo apt update && rosdep update
    rosdep install --from-paths src --ignore-src -y
```
1. Build and Source micro-ROS Tools
```sh
    colcon build
    source install/local_setup.bash
```
1. Run the micro-ROS Agent
   1.  Serial Mode:
    ```sh
        ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    ```
   2.  Wifi Mode:
    ```sh
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ```
## Next
- [Main](../README.md)