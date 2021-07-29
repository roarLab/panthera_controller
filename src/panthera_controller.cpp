#include <panthera_controller/panthera_controller.h>

namespace panthera_controller {
PantheraController::PantheraController()
    : command_struct_(), cmd_vel_timeout_(0.5) {}

bool PantheraController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Get joint names from the parameter server
  std::vector<std::string> front_wheel_names, rear_wheel_names;
  if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
      !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names)) {
    return false;
  }

  if (front_wheel_names.size() != rear_wheel_names.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "#front wheels ("
                                      << front_wheel_names.size() << ") != "
                                      << "#rear wheels ("
                                      << rear_wheel_names.size() << ").");
    return false;
  } else if (front_wheel_names.size() != 2) {
    ROS_ERROR_STREAM_NAMED(
        name_, "#two wheels by axle (left and right) is needed; now : "
                   << front_wheel_names.size() << " .");
    return false;
  } else {
    front_wheel_joints_.resize(front_wheel_names.size());
    rear_wheel_joints_.resize(front_wheel_names.size());
  }

  // Get steering joint names from the parameter server
  std::vector<std::string> front_steering_names, rear_steering_names;
  if (!getWheelNames(controller_nh, "front_steering", front_steering_names) ||
      !getWheelNames(controller_nh, "rear_steering", rear_steering_names)) {
    return false;
  }

  if (front_steering_names.size() != rear_steering_names.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "#left steerings ("
                                      << front_steering_names.size() << ") != "
                                      << "#right steerings ("
                                      << rear_steering_names.size() << ").");
    return false;
  } else if (front_steering_names.size() != 2) {
    ROS_ERROR_STREAM_NAMED(
        name_, "#two steering by axle (left and right) is needed; now : "
                   << front_steering_names.size() << " .");
    return false;
  } else {
    front_steering_joints_.resize(front_steering_names.size());
    rear_steering_joints_.resize(front_steering_names.size());
  }

  // Get reconfigurable joint names from the parameter server
  std::vector<std::string> reconfiguration_joint_names;
  if (!getWheelNames(controller_nh, "reconfiguration",
                     reconfiguration_joint_names)) {
    return false;
  }
  reconfiguration_joints_.resize(reconfiguration_joint_names.size());

  // Twist command related:
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  ROS_INFO_STREAM_NAMED(
      name_, "Velocity commands will be considered old if they are older than "
                 << cmd_vel_timeout_ << "s.");

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits",
                      limiter_lin_x_.has_velocity_limits,
                      limiter_lin_x_.has_velocity_limits);
  controller_nh.param("linear/x/has_acceleration_limits",
                      limiter_lin_x_.has_acceleration_limits,
                      limiter_lin_x_.has_acceleration_limits);
  controller_nh.param("linear/x/has_jerk_limits",
                      limiter_lin_x_.has_jerk_limits,
                      limiter_lin_x_.has_jerk_limits);
  controller_nh.param("linear/x/max_velocity", limiter_lin_x_.max_velocity,
                      limiter_lin_x_.max_velocity);
  controller_nh.param("linear/x/min_velocity", limiter_lin_x_.min_velocity,
                      -limiter_lin_x_.max_velocity);
  controller_nh.param("linear/x/max_acceleration",
                      limiter_lin_x_.max_acceleration,
                      limiter_lin_x_.max_acceleration);
  controller_nh.param("linear/x/min_acceleration",
                      limiter_lin_x_.min_acceleration,
                      -limiter_lin_x_.max_acceleration);
  controller_nh.param("linear/x/max_velocity", limiter_lin_x_.max_jerk,
                      limiter_lin_x_.max_jerk);
  controller_nh.param("linear/x/min_velocity", limiter_lin_x_.min_jerk,
                      -limiter_lin_x_.min_jerk);

  controller_nh.param("linear/y/has_velocity_limits",
                      limiter_lin_y_.has_velocity_limits,
                      limiter_lin_y_.has_velocity_limits);
  controller_nh.param("linear/y/has_acceleration_limits",
                      limiter_lin_y_.has_acceleration_limits,
                      limiter_lin_y_.has_acceleration_limits);
  controller_nh.param("linear/y/has_jerk_limits",
                      limiter_lin_y_.has_jerk_limits,
                      limiter_lin_y_.has_jerk_limits);
  controller_nh.param("linear/y/max_velocity", limiter_lin_y_.max_velocity,
                      limiter_lin_y_.max_velocity);
  controller_nh.param("linear/y/min_velocity", limiter_lin_y_.min_velocity,
                      -limiter_lin_y_.max_velocity);
  controller_nh.param("linear/y/max_acceleration",
                      limiter_lin_y_.max_acceleration,
                      limiter_lin_y_.max_acceleration);
  controller_nh.param("linear/y/min_acceleration",
                      limiter_lin_y_.min_acceleration,
                      -limiter_lin_y_.max_acceleration);
  controller_nh.param("linear/y/max_velocity", limiter_lin_y_.max_jerk,
                      limiter_lin_y_.max_jerk);
  controller_nh.param("linear/y/min_velocity", limiter_lin_y_.min_jerk,
                      -limiter_lin_y_.min_jerk);

  controller_nh.param("angular/z/has_velocity_limits",
                      limiter_ang_z_.has_velocity_limits,
                      limiter_ang_z_.has_velocity_limits);
  controller_nh.param("angular/z/has_acceleration_limits",
                      limiter_ang_z_.has_acceleration_limits,
                      limiter_ang_z_.has_acceleration_limits);
  controller_nh.param("angular/z/max_velocity", limiter_ang_z_.max_velocity,
                      limiter_ang_z_.max_velocity);
  controller_nh.param("angular/z/min_velocity", limiter_ang_z_.min_velocity,
                      -limiter_ang_z_.max_velocity);
  controller_nh.param("angular/z/max_acceleration",
                      limiter_ang_z_.max_acceleration,
                      limiter_ang_z_.max_acceleration);
  controller_nh.param("angular/z/min_acceleration",
                      limiter_ang_z_.min_acceleration,
                      -limiter_ang_z_.max_acceleration);

  // If either parameter is not available, we need to look up the value in the
  // URDF
  bool lookup_track_width =
      !controller_nh.getParam("track_width", track_width_);
  bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);
  bool lookup_wheel_radius =
      !controller_nh.getParam("wheel_radius", wheel_radius_);

  hardware_interface::VelocityJointInterface *const vel_joint_hw =
      robot_hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::PositionJointInterface *const pos_joint_hw =
      robot_hw->get<hardware_interface::PositionJointInterface>();

  // Get the joint object to use in the realtime loop
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i) {
    ROS_INFO_STREAM_NAMED(name_, "Adding front wheel with joint name: "
                                     << front_wheel_names[i]
                                     << " and rear wheel with joint name: "
                                     << rear_wheel_names[i]);
    front_wheel_joints_[i] =
        vel_joint_hw->getHandle(front_wheel_names[i]);  // throws on failure
    rear_wheel_joints_[i] =
        vel_joint_hw->getHandle(rear_wheel_names[i]);  // throws on failure
  }

  // Get the steering joint object to use in the realtime loop
  for (size_t i = 0; i < front_steering_joints_.size(); ++i) {
    ROS_INFO_STREAM_NAMED(name_, "Adding left steering with joint name: "
                                     << front_steering_names[i]
                                     << " and right steering with joint name: "
                                     << rear_steering_names[i]);
    front_steering_joints_[i] =
        pos_joint_hw->getHandle(front_steering_names[i]);  // throws on failure
    rear_steering_joints_[i] =
        pos_joint_hw->getHandle(rear_steering_names[i]);  // throws on failure
  }

  // Get the reconfiguration joint object to use in the realtime loop
  for (size_t i = 0; i < reconfiguration_joints_.size(); ++i) {
    ROS_INFO_STREAM_NAMED(name_, "Adding reconfiguration with joint name: "
                                     << reconfiguration_joint_names[i]);
    reconfiguration_joints_[i] = pos_joint_hw->getHandle(
        reconfiguration_joint_names[i]);  // throws on failure
  }

  sub_command_ = controller_nh.subscribe(
      "cmd_vel", 1, &PantheraController::cmdVelCallback, this);

  return true;
}

void PantheraController::update(const ros::Time &time,
                                const ros::Duration &period) {
  // MOVE ROBOT
  // Retreive current velocity command and time step
  Commands curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_) {
    curr_cmd.lin_x = 0.0;
    curr_cmd.lin_y = 0.0;
    curr_cmd.ang_z = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());

  limiter_lin_x_.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x,
                       cmd_dt);
  limiter_lin_y_.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y,
                       cmd_dt);
  limiter_ang_z_.limit(curr_cmd.ang_z, last0_cmd_.ang_z, last1_cmd_.ang_z,
                       cmd_dt);
  last1_cmd_ = last0_cmd_;
  last0_cmd_ = curr_cmd;

  //////////////////////////////////////////////////////////////
  // Note that, track-width currently is fixed but later it'll
  // change based on desired reconfiguration
  // TODO(phone): Find track width based on current positions of
  // two gaits of Panthera
  //
  const double left_track_width =
      (track_width_ / 2.0) + fabs(reconfiguration_joints_[0].getPosition());
  const double right_track_width =
      (track_width_ / 2.0) + fabs(reconfiguration_joints_[1].getPosition());
  //////////////////////////////////////////////////////////////

  // find wheel velocities in x and y direction
  const double vel_x_wheel_fl =
      curr_cmd.lin_x - (curr_cmd.ang_z * left_track_width);
  const double vel_y_wheel_fl =
      curr_cmd.lin_y + (curr_cmd.ang_z * wheel_base_ / 2.0);
  const double vel_x_wheel_fr =
      curr_cmd.lin_x + (curr_cmd.ang_z * right_track_width);
  const double vel_y_wheel_fr =
      curr_cmd.lin_y + (curr_cmd.ang_z * wheel_base_ / 2.0);

  const double vel_x_wheel_rl =
      curr_cmd.lin_x - (curr_cmd.ang_z * left_track_width);
  const double vel_y_wheel_rl =
      curr_cmd.lin_y - (curr_cmd.ang_z * wheel_base_ / 2.0);
  const double vel_x_wheel_rr =
      curr_cmd.lin_x + (curr_cmd.ang_z * right_track_width);
  const double vel_y_wheel_rr =
      curr_cmd.lin_y - (curr_cmd.ang_z * wheel_base_ / 2.0);

  // Now, find new wheel velocity
  double new_wheel_vel_fl = std::hypot(vel_x_wheel_fl, vel_y_wheel_fl);
  double new_wheel_vel_fr = std::hypot(vel_x_wheel_fr, vel_y_wheel_fr);
  double new_wheel_vel_rl = std::hypot(vel_x_wheel_rl, vel_y_wheel_rl);
  double new_wheel_vel_rr = std::hypot(vel_x_wheel_rr, vel_y_wheel_rr);
  // and steering angle
  double new_wheel_angle_fl = std::atan2(vel_y_wheel_fl, vel_x_wheel_fl);
  double new_wheel_angle_fr = std::atan2(vel_y_wheel_fr, vel_x_wheel_fr);
  double new_wheel_angle_rl = std::atan2(vel_y_wheel_rl, vel_x_wheel_rl);
  double new_wheel_angle_rr = std::atan2(vel_y_wheel_rr, vel_x_wheel_rr);

  // need to normalize angles between -90 and 90
  const double new_wheel_angle_fl_normalized =
      normalize_angle(new_wheel_angle_fl);
  const double new_wheel_angle_fr_normalized =
      normalize_angle(new_wheel_angle_fr);
  const double new_wheel_angle_rl_normalized =
      normalize_angle(new_wheel_angle_rl);
  const double new_wheel_angle_rr_normalized =
      normalize_angle(new_wheel_angle_rr);

  // update wheel velocity directions
  if (fabs(new_wheel_angle_fl_normalized - new_wheel_angle_fl) > 0.00001) {
    new_wheel_vel_fl = -1 * new_wheel_vel_fl;
  }
  if (fabs(new_wheel_angle_fr_normalized - new_wheel_angle_fr) > 0.00001) {
    new_wheel_vel_fr = -1 * new_wheel_vel_fr;
  }
  if (fabs(new_wheel_angle_rl_normalized - new_wheel_angle_rl) > 0.00001) {
    new_wheel_vel_rl = -1 * new_wheel_vel_rl;
  }
  if (fabs(new_wheel_angle_rr_normalized - new_wheel_angle_rr) > 0.00001) {
    new_wheel_vel_rr = -1 * new_wheel_vel_rr;
  }

  front_steering_joints_[0].setCommand(new_wheel_angle_fl_normalized);
  front_steering_joints_[1].setCommand(new_wheel_angle_fr_normalized);
  rear_steering_joints_[0].setCommand(new_wheel_angle_rl_normalized);
  rear_steering_joints_[1].setCommand(new_wheel_angle_rr_normalized);

  ///////////////////////////////////////////////////////////////
  // before sending velocity commands
  // we need to make sure the error between desired and current
  // steering angles are not too large,
  // if so, we don't rotate wheels
  ///////////////////////////////////////////////////////////////

  // get current wheel angles
  const double curr_wheel_angle_fl = front_steering_joints_[0].getPosition();
  const double curr_wheel_angle_fr = front_steering_joints_[1].getPosition();
  const double curr_wheel_angle_rl = rear_steering_joints_[0].getPosition();
  const double curr_wheel_angle_rr = rear_steering_joints_[1].getPosition();

  // if all the errors are less than aprroximately 10 degree
  // TODO: this error threshold should be user-defined parameter
  if (fabs(new_wheel_angle_fl_normalized - curr_wheel_angle_fl) < 0.17 &&
      fabs(new_wheel_angle_fr_normalized - curr_wheel_angle_fr) < 0.17 &&
      fabs(new_wheel_angle_rl_normalized - curr_wheel_angle_rl) < 0.17 &&
      fabs(new_wheel_angle_rr_normalized - curr_wheel_angle_rr) < 0.17) {
    front_wheel_joints_[0].setCommand(new_wheel_vel_fl / wheel_radius_);
    front_wheel_joints_[1].setCommand(new_wheel_vel_fr / wheel_radius_);
    rear_wheel_joints_[0].setCommand(new_wheel_vel_rl / wheel_radius_);
    rear_wheel_joints_[1].setCommand(new_wheel_vel_rr / wheel_radius_);
  } else {
    front_wheel_joints_[0].setCommand(0.0);
    front_wheel_joints_[1].setCommand(0.0);
    rear_wheel_joints_[0].setCommand(0.0);
    rear_wheel_joints_[1].setCommand(0.0);
  }
  ///////////////////////////////////////////////////////////////

  // apply desired positions to gaits
  // TODO: make sure cmds are between min and max limits
  reconfiguration_joints_[0].setCommand(curr_cmd.left_gait);
  reconfiguration_joints_[1].setCommand(curr_cmd.right_gait);
}

double PantheraController::normalize_angle(double angle) {
  const double result = fmod(angle + M_PI_2, M_PI);
  if (result <= 0.0) return result + M_PI_2;
  return result - M_PI_2;
}

void PantheraController::cmdVelCallback(
    const panthera_msgs::TwistWithReconfiguration &msg) {
  if (isRunning()) {
    if (std::isnan(msg.x) || std::isnan(msg.y) || std::isnan(msg.z) ||
        std::isnan(msg.left_frame_position) ||
        std::isnan(msg.right_frame_position)) {
      ROS_WARN(
          "Received NaN in panthera_msgs::TwistWithReconfiguration. Ignoring "
          "command.");
      return;
    }
    command_struct_.ang_z = msg.z;
    command_struct_.lin_x = msg.x;
    command_struct_.lin_y = msg.y;
    command_struct_.left_gait = msg.left_frame_position;
    command_struct_.right_gait = msg.right_frame_position;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Added values to command. "
                               << "Ang: " << command_struct_.ang_z << ", "
                               << "Lin x: " << command_struct_.lin_x << ", "
                               << "Lin y: " << command_struct_.lin_y << ", "
                               << "Stamp: " << command_struct_.stamp);
  } else {
    ROS_ERROR_NAMED(name_,
                    "Can't accept new commands. Controller is not running.");
  }
}

void PantheraController::starting(const ros::Time &time) {
  brake();

  // Register starting time used to keep fixed rate
  // last_state_publish_time_ = time;
}

void PantheraController::stopping(const ros::Time & /*time*/) { brake(); }

void PantheraController::brake() {
  const double vel = 0.0;
  for (size_t i = 0; i < front_wheel_joints_.size(); ++i) {
    front_wheel_joints_[i].setCommand(vel);
    rear_wheel_joints_[i].setCommand(vel);
  }

  const double pos = 0.0;
  for (size_t i = 0; i < front_steering_joints_.size(); ++i) {
    front_steering_joints_[i].setCommand(pos);
    rear_steering_joints_[i].setCommand(pos);
  }
}

bool PantheraController::getWheelNames(ros::NodeHandle &controller_nh,
                                       const std::string &wheel_param,
                                       std::vector<std::string> &wheel_names) {
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list)) {
    ROS_ERROR_STREAM_NAMED(
        name_, "Couldn't retrieve wheel param '" << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    if (wheel_list.size() == 0) {
      ROS_ERROR_STREAM_NAMED(
          name_, "Wheel param '" << wheel_param << "' is an empty list");
      return false;
    }

    for (int i = 0; i < wheel_list.size(); ++i) {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #"
                                                      << i
                                                      << " isn't a string.");
        return false;
      }
    }

    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i) {
      wheel_names[i] = static_cast<std::string>(wheel_list[i]);
    }
  } else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
    wheel_names.push_back(wheel_list);
  } else {
    ROS_ERROR_STREAM_NAMED(
        name_, "Wheel param '"
                   << wheel_param
                   << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}
}  // namespace panthera_controller