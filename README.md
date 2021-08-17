# ohm_mecanum_sim
This package comprises a pygame-based robot simulation for mecanum-driven kinematic concepts.

![Screenshot of Robot Simulator](/images/screenshot_init.png)
![Original Robot swarm](/images/Roboterschwarm.png)

Tests have been performed with ROS melodic, albeit the used python versions differ. While ROS melodic uses python2.x, the simulator need python3. The  reason is, that ROS noetic will require python3. In order to make the simulator work with ROS melodic, you can follow the installation hints below.

## Prerequisites on Ubuntu 18.04
Installing python3 aside python2 can be done with pip:
```console
user@machine:~$ sudo apt install python3 python-pip python3-pip python-catkin-tools
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
user@machine:~$ pip install -U defusedxml rospkg pygame pycryptodomex gnupg pyside2 pydot numpy
```
## Starting the simulator
Go to your catkin workspace / execute the following commands (replace the path to your catkin workspace accordingly):
```console
user@machine:~$ roscore &
user@machine:~$ cd workspace/catkin_ws
user@machine:~/workspace/catkin_ws$ catkin_make
user@machine:~/workspace/catkin_ws$ source devel/setup.bash
user@machine:~/workspace/catkin_ws$ python3 src/ohm_robot_swarm_sim/scripts/ohm_robot_swarm_sim_node.py
```

## Moving the robots
Use Rosservice to let the robots move around:
- Dispersion
```console
user@machine:~$ rosservice call /dispersion
```
- Assemble
```console
user@machine:~$ rosservice call /assemble
```

- ... or write your own node.
