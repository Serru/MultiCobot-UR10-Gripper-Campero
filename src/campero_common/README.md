# campero_common

Common packages of the Campero: URDF description of the Campero platform messages and other files for simulation.

---

## Packages

### campero_control

This package contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager. It allows to launch the joint controllers for the Campero (4 axes skid steering + 2 axes ptz), Campero OMNI (4 axes skid steering, 4 axes swerve drive).

The Campero simulation stack follows the gazebo_ros controller manager scheme described in
http://gazebosim.org/wiki/Tutorials/1.9/ROS_Control_with_Gazebo

### campero_description

The urdf, meshes, and other elements needed in the description are contained here. The standard camera configurations have been included (w/wo sphere_camera, w/wo axis_camera, etc.). This package includes the description of the Campero, Campero STEEL (and HL versions) mobile platforms.
The package includes also some launch files to publish the robot state and to test the urdf files in rviz.

### campero_localization

This package contains launch files to use the EKF of the robot_localization package with the Campero robots. It contains a node to subscribe to gps data and publish it as odometry to be used as an additional source for odometry.

### campero_navigation

This package contains all the configuration files needed to execute the AMCL and SLAM navigation algorithms in simulation.

### campero_pad

This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform including speed level control. The joystick output is feed to a mux (http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)

The node allows to load different types of joysticks (PS4, PS3, Logitech, Thrustmaster). New models can be easily added by creating new .yaml files. If modbus_io node is available, the digital outputs (ligths, axes, etc.) can also be controlled with the pad. If ptz camera is available, the pan-tilt-zoom can also be commanded with the pad. 
