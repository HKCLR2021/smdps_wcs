#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_conveyor(
  const uint16_t speed, 
  const bool stop_by_ph, 
  const bool fwd, 
  const bool ctrl)
{
  bool success = true;

  if (!ctrl) 
  {
    success &= call_co_write(0x6089, 0x0, 0);
    if (success)
      RCLCPP_INFO(this->get_logger(), "Stop the conveyor");

    return success;
  }

  success &= call_co_write(0x6080, 0x0, speed > 3000 ? 3000 : speed);
  success &= call_co_write(0x6081, 0x0, stop_by_ph ? 1 : 0);
  success &= call_co_write(0x6082, 0x0, fwd ? 0 : 1);
  success &= call_co_write(0x6089, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "moving the conveyor %s", stop_by_ph ? "with stop by photoelectric sensor" : "");
  
  return success;
}

bool PackagingMachineNode::read_conveyor_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6088>(data);
}

bool PackagingMachineNode::read_conveyor_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6089>(data);
}