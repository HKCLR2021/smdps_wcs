#include "packaging_machine_control_system/packaging_machine_node.hpp"

void PackagingMachineNode::order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PackagingOrder::Feedback>();
  auto& curr_order_status = feedback->curr_order_status;
  auto& are_drugs_fallen = feedback->are_drugs_fallen;
  auto result = std::make_shared<PackagingOrder::Result>();

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  RCLCPP_INFO(this->get_logger(), "======== packaging sequence 1 ==========");

  ctrl_material_box_gate(MTRL_BOX_GATE_OPEN);
  wait_for_material_box_gate(MTRL_BOX_GATE_OPEN_STATE);

  rclcpp::sleep_for(DELAY_MTRL_BOX_GATE);

  ctrl_material_box_gate(MTRL_BOX_GATE_CLOSE);
  wait_for_material_box_gate(MTRL_BOX_GATE_CLOSE_STATE);

  are_drugs_fallen = true;
  RCLCPP_INFO(this->get_logger(), "Set are_drugs_fallen to True");
  goal_handle->publish_feedback(feedback);

  lock.lock();
  status_->waiting_material_box = false;
  lock.unlock();

  UnbindRequest msg;
  msg.packaging_machine_id = status_->packaging_machine_id;
  msg.order_id = goal->order_id;
  msg.material_box_id = goal->material_box_id;
  unbind_mtrl_box_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published an unbind material box id request");

  RCLCPP_INFO(this->get_logger(), "========== packaging sequence 2 ==========");

  auto perform_dis_push_pull = [&]() {
    rclcpp::sleep_for(DELAY_PKG_DIS_WAIT_PRINTER);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN_FACTOR, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis();

    ctrl_pkg_dis(PKG_DIS_UNFEED_LEN, PKG_DIS_UNFEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis();

    ctrl_cutter(CUTTER_CLIP);

    ctrl_squeezer(SQUEEZER_ACTION_PUSH, MOTOR_ENABLE);
    wait_for_squeezer();

    rclcpp::sleep_for(DELAY_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PULL, MOTOR_ENABLE);
    wait_for_squeezer();

    ctrl_cutter(CUTTER_RELEASE);
  };

  auto print_empty_pkg = [&]() {
    PackageInfo _msg;
    std::vector<std::string> cmd = get_print_label_cmd(_msg);
    printer_->runTask(cmd);
    RCLCPP_INFO(this->get_logger(), "printed a empty package");
  };

  std::queue<size_t> to_be_printed;
  std::queue<size_t> printed;
  size_t postfix = PKG_POSTFIX;

  for (size_t i = 0; i < CELLS; i++)
  {
    if (!goal->print_info[i].en_name.empty()) // FIXME
      to_be_printed.push(i);
  }
  RCLCPP_INFO(this->get_logger(), "to_be_printed size: %ld", to_be_printed.size());

  // make sure the bag is tight
  rclcpp::sleep_for(DELAY_ORDER_START_WAIT_FOR);
  ctrl_pkg_dis(status_->package_length / 4, PKG_DIS_FEED_DIR, MOTOR_ENABLE); 
  wait_for_pkg_dis();

  for (size_t i = 0; i < PKG_PREFIX; i++)
  {
    if (to_be_printed.empty())
    {
      if (postfix > 0)
        postfix--;

      print_empty_pkg();
    }
    else
    {    
      std::vector<std::string> cmd = get_print_label_cmd(goal->print_info[to_be_printed.front()]);
      printer_->runTask(cmd);
      RCLCPP_INFO(this->get_logger(), "printed a order %ld package", printed.front());

      printed.push(to_be_printed.front());
      to_be_printed.pop();
    }

    perform_dis_push_pull();
  }
  RCLCPP_INFO(this->get_logger(), "Printed %d prefix", PKG_PREFIX);

  for (uint8_t day = 0; day < DAYS; day++)
  {
    RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@ Day: %d @@@@@@@@@@", day);

    rclcpp::sleep_for(DELAY_GENERAL_STEP);
    ctrl_roller(1, 0, MOTOR_ENABLE);
    wait_for_roller();

    auto index_exist_in_printed = [&]() {
      for (uint8_t cell = 0; cell < CELLS_PER_DAY; cell++)
      {
        const size_t index = CELLS_PER_DAY * day + cell;
        if (!printed.empty() && printed.front() == index)
          return true;
      }
      return false;
    };

    if (!index_exist_in_printed())
      continue;

    for (uint8_t cell = 0; cell < CELLS_PER_DAY; cell++)
    {
      const size_t index = CELLS_PER_DAY * day + cell;
      RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@ index: %ld @@@@@@@@@@", index);

      rclcpp::sleep_for(DELAY_GENERAL_STEP);
      ctrl_pill_gate(PILL_GATE_WIDTH, PILL_GATE_OPEN_DIR, MOTOR_ENABLE);
      wait_for_pill_gate();

      if (!printed.empty() && printed.front() == index)
      {
        printed.pop();
      
        if (to_be_printed.empty())
        {
          if (postfix > 0)
            postfix--;

          print_empty_pkg();
        }
        else
        {        
          std::vector<std::string> cmd = get_print_label_cmd(goal->print_info[to_be_printed.front()]);
          printer_->runTask(cmd);
          RCLCPP_INFO(this->get_logger(), "printed a order %ld package", printed.front());

          printed.push(to_be_printed.front());
          to_be_printed.pop();
        }

        perform_dis_push_pull();
      }

      curr_order_status[index] = true;
      goal_handle->publish_feedback(feedback);
    }

    rclcpp::sleep_for(DELAY_GENERAL_STEP);
    ctrl_pill_gate(PILL_GATE_WIDTH * NO_OF_PILL_GATES * PILL_GATE_CLOSE_MARGIN_FACTOR, PILL_GATE_CLOSE_DIR, MOTOR_ENABLE);
    wait_for_pill_gate();
  }
  RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> completed %d cells <<<<<<<<<<", CELLS);

  for (uint8_t i = 0; i < postfix; i++)
  {
    print_empty_pkg();
    perform_dis_push_pull();
  }

  rclcpp::sleep_for(DELAY_GENERAL_STEP);
  ctrl_roller(0, 1, MOTOR_ENABLE);
  wait_for_roller();
  
  uint8_t num_printed = 0;
  for (size_t i = 0; i < CELLS; i++)
  {
    if (!goal->print_info[i].en_name.empty()) // FIXME
      num_printed++;
  }

  uint32_t remain_package = read_ribbon("package");
  uint32_t remain_thermal = read_ribbon("thermal");

  write_ribbon("package", remain_package - (num_printed * status_->package_length));
  write_ribbon("thermal", remain_thermal - (num_printed * status_->package_length));

  if (rclcpp::ok()) 
  {
    printer_.reset();
    RCLCPP_INFO(this->get_logger(), "printer destroyed");
    RCLCPP_INFO(this->get_logger(), "postfix: %ld", postfix);

    result->order_result = curr_order_status;
    goal_handle->succeed(result);

    status_->packaging_machine_state = PackagingMachineStatus::IDLE;
    
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void PackagingMachineNode::skip_order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal w/ skip operation");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PackagingOrder::Feedback>();
  auto& curr_order_status = feedback->curr_order_status;
  auto& are_drugs_fallen = feedback->are_drugs_fallen;
  auto result = std::make_shared<PackagingOrder::Result>();

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  are_drugs_fallen = true;
  RCLCPP_INFO(this->get_logger(), "Set are_drugs_fallen to True");
  goal_handle->publish_feedback(feedback);

  lock.lock();
  status_->waiting_material_box = false;
  lock.unlock();

  UnbindRequest msg;
  msg.packaging_machine_id = status_->packaging_machine_id;
  msg.order_id = goal->order_id;
  msg.material_box_id = goal->material_box_id;
  unbind_mtrl_box_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published an unbind material box id request");

  if (rclcpp::ok()) 
  {
    printer_.reset();
    RCLCPP_INFO(this->get_logger(), "printer destroyed");

    result->order_result = curr_order_status;
    goal_handle->succeed(result);
    
    status_->packaging_machine_state = PackagingMachineStatus::IDLE;
    
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
