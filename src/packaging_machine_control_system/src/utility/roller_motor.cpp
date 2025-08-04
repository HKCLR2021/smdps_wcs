#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_roller(
  const uint8_t days, 
  const bool home, 
  const bool ctrl)
{
  bool success = true;

  if (!ctrl) 
  {
    success &= call_co_write(0x6039, 0x0, 0);
    if (success)
      RCLCPP_INFO(this->get_logger(), "Stop the roller");

    return success;
  }

  if (home) 
  {
    success &= call_co_write(0x6030, 0x0, 1); // must be 1 step
    success &= call_co_write(0x6037, 0x0, 1); // set mode 1 to go home
  }  else {
    success &= call_co_write(0x6030, 0x0, days > DAYS ? DAYS : days);
    success &= call_co_write(0x6037, 0x0, 0); // set mode 0 to go X day(s)
  }

  success &= call_co_write(0x6032, 0x0, 0); // direction must be 0 
  success &= call_co_write(0x6039, 0x0, 1);

  if (success)
  {
    if (home)
      RCLCPP_INFO(this->get_logger(), "moving the roller to home");
    else
      RCLCPP_INFO(this->get_logger(), "moving the roller to %s day(s)", std::to_string(days).c_str());
  }

  return success;
}

bool PackagingMachineNode::read_roller_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6038>(data);
}

bool PackagingMachineNode::read_roller_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6039>(data);
}
