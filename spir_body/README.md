# spir_body

## Overview

This package consists of nodes that deal with the control and functioning of the robot body. It receives thruster commands from the joystick (for general control of the frame) and from the IMU (for roll and pitch stabilisation), scales them based on the maximum power capacity of the system and sends the final thruster commands over UDP to the arduino. 

This package also stores the urdf information and rviz model of the robot body (i.e. excluding the payload). The rviz model is a mesh model used to visualise the robot and can be used for collision detection if a collission mesh is added. The urdf information stores the relative frames of the different parts of the robot e.g. Centre of mass, thrusters, payload, frame etc. 


The spir_body package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors: Brendan Emery, Luke Coffey**

**Contact:** Brendan.Emery@student.uts.edu.au, Luke.Coffey@student.uts.edu.au

**Affiliation: Centre for Autonomous Systems (CAS), University of Technology, Sydney (UTS)**

***
## Installation

### Dependencies

 - [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
 -
 -
 -
 
 
 
### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile using:

	cd catkin_workspace/src
	git clone https://codeine.research.uts.edu.au/spir/ros.git
	cd ../
	catkin_make

Note. This will clone the entire workspace.

***
## Launch Files

### spir_body_core

This launch file starts/loads all the things necessary for the spir_body to operate such as running the two robot cameras, starting the IMU stabilisation control and running the robot/joint state publishers. Generally this file is included in a parent launch file.

#### Node diagram

![spir_body_core](https://codeine.research.uts.edu.au/spir/ros/raw/master/indigo/spir_body/doc/images/spir_body_core.jpg)

***
### joy_control

This launch file starts/loads all the things necessary to run joystick control of the spir_body. This includes the spir_body_core launch file and also all the nodes that convert joystick commands through to thruster values.

#### Node diagram

![spir_body_core](https://codeine.research.uts.edu.au/spir/ros/raw/master/indigo/spir_body/doc/images/joy_control.jpg)

***
### spir_body_core_pid

This launch file is a replica of the spir_body_core launch file except instead of using a robust integral algorithm for the stabilisation control, it uses a PID algorithm.

#### Node diagram

See spir_body_core

***
### joy_control_pid

This launch file is a replica of the joy_control launch file except instead of using a robust integral algorithm for the stabilisation control, it uses a PID algorithm.

#### Node diagram

See joy_control

***
## Nodes

### [joy_to_wrench](joy__to__wrench_8cpp.html)

This node subscribes to a joystick topic and converts it into a stamped wrench, representing linear and angular forces along the x, y and z axes of the Centre of Mass (COM) of the robot. 

#### Subscribed Topics

* ```joy``` ([sensor_msgs/Joy])

	The joy topic from the joystick. The node that publishes this "joy" topic is a pre-installed dependency that is run in the launch file.

#### Published Topics

* ```wrench``` ([geometry_msgs/WrenchStamped])

	The wrench topic on which the linear and angular forces at the COM (based only on the joystick) are published.

#### Services

None

#### Parameters

* ```max_vx``` (double, default: "136")

* ```max_vy``` (double, default: "136")

* ```max_vz``` (double, default: "136")

* ```max_wx``` (double, default: "30")

* ```max_wy``` (double, default: "30")

* ```max_wz``` (double, default: "30")

These parameters are the scaling factors that are used to scale the joystick inputs (which are between [-1,1]) to produce a force value

***
### [wrench_to_thrust](wrench__to__thrust_8cpp.html)

This node converts force commands at the centre of mass of the robot to individual thruster force commands.

#### Subscribed Topics

* ```wrench``` ([geometry_msgs/WrenchStamped])

	The wrench topic on which the centre of mass force commands from the joystick are published.

#### Published Topics

* ```thrust_command``` (spir_body/ThrustStamped)

	The thrust_command topic on which the individual thruster force commands are published.

#### Services

None

#### Parameters

None

***
### [imu_control](imu__control_8cpp.html)

This node uses the a robust integral control algorithm to calculate thruster forces required to stabilise the robot about roll and pitch. 

#### Subscribed Topics

* ```imu/data``` ([sensor_msgs/Imu])

	The imu/data topic gets the orientation, angular velocity and angular acceleration values from the IMU.	

#### Published Topics

* ```thrust_command``` (spir_body/ThrustStamped)

	The thrust_command topic publishes the individual thruster forces required to stabilise the robot. 

#### Services

None

#### Parameters

None

***
### [imu_control_pid](imu__control__pid_8cpp.html)

This node uses the a PID algorithm to calculate thruster forces required to stabilise the robot about roll and pitch. 

#### Subscribed Topics

* ```imu/data``` ([sensor_msgs/Imu])

	The imu/data topic gets the orientation, angular velocity and angular acceleration values from the IMU.	

#### Published Topics

* ```thrust_command``` (spir_body/ThrustStamped)

	The thrust_command topic publishes the individual thruster forces required to stabilise the robot. 

#### Services

None

#### Parameters

None


***
### [merge_thruster_commands](merge__thruster__commands_8cpp.html)


Combines the thruster commands from the joystick and the imu control and scales them based on the maximum power capacity of the system and also each individual thruster. It prioritises allocating thrust to IMU commands, although the user can set a minimum thrust allocation that must be given to the joystick. It then outputs this scaled and combined thruster value.

#### Subscribed Topics

* ```thrust_command_1``` (spir_body/ThrustStamped)

	The thrust_command_1 topic on which the joystick, individual thruster commands are published. This topic is remapped in the launch file to thrust_command_joy.

* ```thrust_command_2``` (spir_body/ThrustStamped)

	The thrust_command_2 topic on which the IMU, individual thruster stabilisation commands are published. This topic is remapped in the launch file to thrust_command_imu.

#### Published Topics

* ```thruster_commands_out``` (spir_body/ThrustStamped)

	The thruster_commands_out topic publishes the combined and scaled individual thruster values.

#### Services

None

#### Parameters

* ```max_thruster_val``` (double, default: "17.0")
	
	This parameter defines the maximum thrust that each individual thruster can safely output to prevent overdrawing current.

* ```available_power``` (double, default: "600.0")
	
	This parameter defines the maximum power that is available to the system that the thrusters can draw from.

* ```min_joy_allocation``` (double, default: "0.0")

	This parameter defines the minimum thrust value (as a fraction of the total thrust available) that must always be given to the joystick (if it requires it). This is to ensure that if the stabilisation needs to draw all the power, the robot can still be controlled using the joystick.  

***
### [thruster_interface](thruster__interface_8cpp.html)

Converts the individual thruster commands into pwm and sends these values to the arduino over UDP.

#### Subscribed Topics

* ```thruster_commands_out``` (spir_body/ThrustStamped)

	The thruster_commands_out topic on which the combined thruster values from the joystick and IMU are published.

#### Services

None

#### Parameters

* ```max_thruster_val``` (double, default: "17.0")
	
	This parameter defines the maximum thrust that each individual thruster can safely output to prevent overdrawing current.

* ```power_supply_wattage``` (double, default: "600.0")
	
	This parameter defines the maximum power that is available to the system that the thrusters can draw from.

***
### [thrust_markers](thrust__markers_8cpp.html)

Publishes RVIZ markers corresponding to thrust values at the Centre of Mass (COM).

#### Subscribed Topics

* ```thrust_command``` (spir_body/ThrustStamped)

	The thrust_commands_out topic on which the thrust values at the COM are published.

#### Published Topics

* ```thrust_visualization``` ([visualization_msgs/Marker])

	The thrust_visualization topic publishes information that RVIZ uses to visualise the thrust values at the COM (shows up as arrows pointing in the direction of the force/moment and whose size is proportional to the magnitude of the force)

#### Services

None

#### Parameters

* ```arrow_scale``` (double, default: "0.1")
	
	This parameter scales the size of the arrow in RVIZ.

***
## Bugs & Feature Requests

There are no known bugs. Please report bugs and request features by emailing the author.

[ROS]: http://www.ros.org
[geometry_msgs/WrenchStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html
[visualization_msgs/Marker]: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
