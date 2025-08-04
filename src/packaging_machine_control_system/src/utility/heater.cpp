#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_heater(const bool on)
{
  bool success = write_heater(on ? 1 : 0);
  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the heater", on ? "turn on" : "turn off");

  return success;
}

bool PackagingMachineNode::write_heater(const uint32_t data)
{
  return call_co_write(0x6003, 0x0, data);
}

bool PackagingMachineNode::read_heater(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6003>(data);
}
