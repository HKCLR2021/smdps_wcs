#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::ctrl_squeezer(
  const bool squeeze, 
  const bool ctrl)
{
  bool success = true;

  if (!ctrl) 
  {
    success &= call_co_write(0x6079, 0x0, 0);
    if (success)
      RCLCPP_INFO(this->get_logger(), "Stop the squeezer");

    return success;
  }

  success &= call_co_write(0x6070, 0x0, SQUEEZER_SPEED);

  if (squeeze) 
  {
    success &= call_co_write(0x6072, 0x0, 0);
    success &= call_co_write(0x6073, 0x0, 1);
  }  
  else 
  {
    success &= call_co_write(0x6072, 0x0, 1);
    success &= call_co_write(0x6073, 0x0, 0);
  }
  // rclcpp::sleep_for(DELAY_CO_L);

  success &= call_co_write(0x6079, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the squeezer", squeeze ? "push" : "pull");

  return success;
}

bool PackagingMachineNode::read_squeezer_state(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6078>(data);
}

bool PackagingMachineNode::read_squeezer_ctrl(std::shared_ptr<uint32_t> data)
{
  return read_co<0x6079>(data);
}
