#include "packaging_machine_control_system/packaging_machine_node.hpp"

// ===================================== Action =====================================
rclcpp_action::GoalResponse PackagingMachineNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PackagingOrder::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "print_info size: %lu", goal->print_info.size());
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  
  if (info_->temperature <= MIN_TEMP)
  {
    RCLCPP_ERROR(this->get_logger(), "Temperature <= %d", MIN_TEMP);
    return rclcpp_action::GoalResponse::REJECT;
  }

  uint32_t remain_package = read_ribbon("package");
  uint32_t remain_thermal = read_ribbon("thermal");

  if (remain_package + PackagingMachineStatus::REMAIN_MARGIN < goal->print_info.size() * status_->package_length)
  {
    RCLCPP_ERROR(this->get_logger(), "The remain package length may not eneugh to handle the order. [Remain: %d]", remain_package);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (remain_thermal + PackagingMachineStatus::REMAIN_MARGIN < goal->print_info.size() * status_->package_length)
  {
    RCLCPP_ERROR(this->get_logger(), "The remain thermal length may not eneugh to handle the order. [Remain: %d]", remain_thermal);
    return rclcpp_action::GoalResponse::REJECT;
  }

  lock.lock();
  status_->packaging_machine_state = PackagingMachineStatus::BUSY;
  status_->waiting_material_box = true;
  lock.unlock();

  ctrl_stopper(STOPPER_PROTRUDE);
  wait_for_stopper(STOPPER_PROTRUDE_STATE);

  RCLCPP_INFO(this->get_logger(), "set packaging_machine_state to BUSY");
  RCLCPP_INFO(this->get_logger(), "set conveyor_state to UNAVAILABLE");

  init_printer();
  init_printer_config();

  uint16_t retry = 0;
  const uint8_t MAX_RETIRES = 120;
  rclcpp::Rate loop_rate(1s); 
  for (; retry < MAX_RETIRES && rclcpp::ok(); retry++) 
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for the material box, conveyor photoelectic: %s", info_->conveyor ? "1" : "0");
    
    if (!info_->conveyor) 
    {
      ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_DISABLE);
      RCLCPP_INFO(this->get_logger(), "Checking conveyor photoelectric senser: %s", info_->conveyor ? "1" : "0");
      break;
    }
    loop_rate.sleep();
  }

  if (retry >= MAX_RETIRES)
  {
    RCLCPP_INFO(this->get_logger(), "retry(%d) >= MAX_RETIRES", retry);
    status_->packaging_machine_state = PackagingMachineStatus::IDLE;
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request with order %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PackagingMachineNode::handle_cancel(
  const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void) goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PackagingMachineNode::handle_accepted(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (status_->skip_packaging)
    std::thread{std::bind(&PackagingMachineNode::skip_order_execute, this, _1), goal_handle}.detach();
  else
    std::thread{std::bind(&PackagingMachineNode::order_execute, this, _1), goal_handle}.detach();
}