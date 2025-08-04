#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_cutter(const bool cut)
{
  bool success = write_cutter(cut ? 1 : 0);
  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the cutter", cut ? "Switch-on" : "Switch-off");

  return success;
}

bool PackagingMachineNode::write_cutter(const uint32_t data)
{
  return call_co_write(0x6052, 0x0, data);
}

bool PackagingMachineNode::read_cutter(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6056>(data);
}
