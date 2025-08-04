#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_pill_gate(
  const float length, 
  const bool open, 
  const bool ctrl)
{
  bool success = true;

  success &= call_co_write(0x6021, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PILL_GATE_RADIUS)));
  success &= call_co_write(0x6022, 0x0, open ? 1 : 0);
  success &= call_co_write(0x6029, 0x0, ctrl ? 1 : 0);

  if (success)
    RCLCPP_INFO(this->get_logger(), "%s pill gate: %.2fmm", open ? "Open" : "Close", length);

  return success;
}

bool PackagingMachineNode::read_pill_gate_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6028>(data);
}

bool PackagingMachineNode::read_pill_gate_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6029>(data);
}
