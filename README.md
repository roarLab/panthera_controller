# panthera_controller

## Overview

ROS Controller for Panthera, which is a four-wheeled independent steering drive robot with reconfigurable body expansion mechanism. Control is in the form of custom ros message (which extends the standard velocity command with reconfigurable ability) to the the geometic center of the robot and then splits into four velocity-controlled wheels, four position-controlled steering joints and two reconfigurable gaits of Panthera. This controller works with both x and y component of the linear velocity and also together with z component of the angular velocity. Panthera custom Twist message also involves additional two fields for controlling positions of two reconfigurable gaits.

## Features
- Realtime-safe implementation
- Odometry publishing (TODO)
- Task-space velocity, acceleration and jerk limits
- Automatic stop after command time-out

## Dependencies

- [panthera_msgs](https://github.com/roarLab/panthera_msgs)

## Subscribed Topics

cmd_vel (panthera_msgs/TwistWithReconfiguration)
: Velocity and Position command 