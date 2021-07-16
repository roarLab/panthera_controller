#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.hpp>

namespace panthera_controller {

class PantheraController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::VelocityJointInterface,
          hardware_interface::PositionJointInterface> {
 public:
  PantheraController();

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
};

PLUGINLIB_EXPORT_CLASS(panthera_controller::PantheraController,
                       controller_interface::ControllerBase);
}  // namespace panthera_controller
