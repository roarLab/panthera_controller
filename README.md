# panthera_controller

## Overview

ROS Controller for Panthera, which is a four-wheeled independent steering drive robot with reconfigurable body expansion mechanism. Control is in a form of standard velocity command (geometry_msgs/Twist) to the geometic center of the robot and then splits into four velocity-controlled wheels and four position-controlled steering joints. This controller works with both x and y component of the linear velocity and also together with z component of the angular velocity.

> Implementation of position controller for expanding two reconfigurable gaits of Panthera is in progress.

## Features
- Realtime-safe implementation
- Odometry publishing (TODO)
- Task-space velocity, acceleration and jerk limits
- Automatic stop after command time-out

## Subscribed Topics

cmd_vel (geometry_msgs/Twist)
: Velocity command