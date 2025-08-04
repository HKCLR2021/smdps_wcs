#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_stopper(const bool protrude)
{
  bool success = write_stopper(protrude ? 0 : 1);
  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the stopper", protrude ? "protrude" : "sunk");

  return success;
}

bool PackagingMachineNode::write_stopper(const uint32_t data)
{
  return call_co_write(0x6050, 0x0, data);
}

bool PackagingMachineNode::read_stopper(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6054>(data);
}