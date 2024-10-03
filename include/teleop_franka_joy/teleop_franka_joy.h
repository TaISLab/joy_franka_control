/*

ROS Package: joy_franka_control
File: teleop_franka_joy.h
Author: Rodrigo Castro Ochoa, rcastro@uma.es, University of Málaga.

Description:
Teleoperation of the cartesian impedance controller for the Franka Emika Panda manipulator using a joystick. 
This node subscribes to the topic /Joy and publishes a PoseStamped message in the topic /cartesian_impedance_example_controller/equilibrium_pose.

Credits:
This code is based on the teleop_twist_joy project, available at: http://wiki.ros.org/teleop_twist_joy

*/

#ifndef TELEOP_FRANKA_JOY_TELEOP_FRANKA_JOY_H
#define TELEOP_FRANKA_JOY_TELEOP_FRANKA_JOY_H

namespace ros { class NodeHandle; }

namespace teleop_franka_joy
{

/**
 * Class implementing a basic Joy -> PoseStamped translation.
 */
class TeleopFrankaJoy
{
public:
  TeleopFrankaJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace teleop_franka_joy

#endif  // TELEOP_FRANKA_JOY_TELEOP_FRANKA_JOY_H
