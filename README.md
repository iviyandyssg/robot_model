#### Prerequisites

- I have tested on below version.

  - Ubuntu 22.04

  - ROS2 Humble

  - Gazebo Classic 11.10.2

- Install related ROS2 package

```
$ sudo apt install ros-$ROS_DISTRO-gazebo-*
```

#### How to use

1. Clone repository to your ROS workspace

```
$ mkdir -p colcon_ws/src
$ cd ~colcon_ws/src
$ git clone https://github.com/iviyandyssg/robot_model.git
```

2. Then building

```
$ cd ~/colcon_ws
$ colcon build --packages-select robot_model
```

3. Source the workspace

```
$ source ~/colcon_ws/install/setup.bash
```

4. Run

- Task1

```
$ ros2 launch robot_model task1.launch.py
```

- Task2

```
$ ros2 launch robot_model task2.launch.py
```

- Task3

```
$ ros2 launch robot_model task3.launch.py
```

You should see a running effect similar to the following

<img src=".\example\task1.png" style="zoom:30%;" />

<img src=".\example\task2.png" style="zoom:30%;" />

<img src=".\example\task3.png" style="zoom:30%;" />

5. You can control the robot through topic **/cmd_vel** of type **geometry_msgs::msg::Twist** .

