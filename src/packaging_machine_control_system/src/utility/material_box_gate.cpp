#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_material_box_gate(const bool open)
{
  bool success = write_material_box_gate(open ? 1 : 0);
  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the material box gate", open ? "Open" : "Close");

  return success;
}

bool PackagingMachineNode::write_material_box_gate(const uint32_t data)
{
  return call_co_write(0x6051, 0x0, data);
}

bool PackagingMachineNode::read_material_box_gate(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6055>(data);
}