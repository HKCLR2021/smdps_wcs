#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_pkg_dis(
  const float length, 
  const bool feed, 
  const bool ctrl)
{
  bool success = true;

  success &= call_co_write(0x6011, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PKG_DIS_RADIUS)));
  success &= call_co_write(0x6012, 0x0, feed ? 1 : 0); // Set to 0 to feed the package out
  success &= call_co_write(0x6019, 0x0, ctrl ? 1 : 0);

  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the package: %.2fmm", feed ? "feed" : "unfeed", length);
  
  return success;
}

bool PackagingMachineNode::read_pkg_dis_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6018>(data);
}

bool PackagingMachineNode::read_pkg_dis_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6019>(data);
}
