#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_pkg_len(
  const uint8_t level, 
  const bool ctrl)
{
  bool success = true;
  if (!ctrl) 
  {
    success &= call_co_write(0x6049, 0x0, 0);
    return success;
  }

  success &= call_co_write(0x6040, 0x0, 1); // move 1 step

  switch (level)
  {
  case 1:
    success &= call_co_write(0x6042, 0x0, 1); // moving upward
    break;
  case 2:
    success &= call_co_write(0x6042, 0x0, 0); // moving downward
    break;
  default:
    return ctrl_pkg_len(0, 0);
    break;
  }

  success &= call_co_write(0x6049, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "moving the pkg len to level ???"); // FIXME
  
  return success;
}

bool PackagingMachineNode::read_pkg_len_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6048>(data);
}

bool PackagingMachineNode::read_pkg_len_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6049>(data);
}