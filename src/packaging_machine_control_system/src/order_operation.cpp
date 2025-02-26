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

  std::this_thread::sleep_for(DELAY_MTRL_BOX_GATE);

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
  RCLCPP_INFO(this->get_logger(), "Published a unbind material box id request");

  RCLCPP_INFO(this->get_logger(), "========== packaging sequence 2 ==========");

  auto perform_dis_push_pull = [&]() {
    std::this_thread::sleep_for(DELAY_PKG_DIS_WAIT_PRINTER);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN_FACTOR, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis(MotorStatus::IDLE);

    ctrl_squeezer(SQUEEZER_ACTION_PUSH, MOTOR_ENABLE);
    wait_for_squeezer(MotorStatus::IDLE);

    std::this_thread::sleep_for(DELAY_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PULL, MOTOR_ENABLE);
    wait_for_squeezer(MotorStatus::IDLE);
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
  std::this_thread::sleep_for(DELAY_ORDER_START_WAIT_FOR);
  ctrl_pkg_dis(status_->package_length / 4, PKG_DIS_FEED_DIR, MOTOR_ENABLE); 
  wait_for_pkg_dis(MotorStatus::IDLE);

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

    std::this_thread::sleep_for(DELAY_GENERAL_STEP);
    ctrl_roller(1, 0, MOTOR_ENABLE);
    wait_for_roller(MotorStatus::IDLE);

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

      std::this_thread::sleep_for(DELAY_GENERAL_STEP);
      ctrl_pill_gate(PILL_GATE_WIDTH, PILL_GATE_OPEN_DIR, MOTOR_ENABLE);
      wait_for_pill_gate(MotorStatus::IDLE);

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

    std::this_thread::sleep_for(DELAY_GENERAL_STEP);
    ctrl_pill_gate(PILL_GATE_WIDTH * NO_OF_PILL_GATES * PILL_GATE_CLOSE_MARGIN_FACTOR, PILL_GATE_CLOSE_DIR, MOTOR_ENABLE);
    wait_for_pill_gate(MotorStatus::IDLE);
  }
  RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> completed %d cells <<<<<<<<<<", CELLS);

  for (uint8_t i = 0; i < postfix; i++)
  {
    print_empty_pkg();
    perform_dis_push_pull();
  }

  std::this_thread::sleep_for(DELAY_GENERAL_STEP);
  ctrl_roller(0, 1, MOTOR_ENABLE);
  wait_for_roller(MotorStatus::IDLE);
  
  // ctrl_cutter(1);
  // std::this_thread::sleep_for(DELAY_GENERAL_VALVE);
  // ctrl_cutter(0);

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

void PackagingMachineNode::init_packaging_machine(void)
{
  status_->packaging_machine_state = PackagingMachineStatus::BUSY;

  RCLCPP_INFO(this->get_logger(), "init_packaging_machine start");
  ctrl_heater(HEATER_ON);
  std::this_thread::sleep_for(DELAY_GENERAL_STEP);

  ctrl_stopper(STOPPER_PROTRUDE);
  wait_for_stopper(STOPPER_PROTRUDE_STATE);
  
  ctrl_material_box_gate(MTRL_BOX_GATE_OPEN);
  wait_for_material_box_gate(MTRL_BOX_GATE_OPEN_STATE);

  std::this_thread::sleep_for(DELAY_MTRL_BOX_GATE);

  ctrl_material_box_gate(MTRL_BOX_GATE_CLOSE);
  wait_for_material_box_gate(MTRL_BOX_GATE_CLOSE_STATE);

  ctrl_stopper(STOPPER_SUNK);
  wait_for_stopper(STOPPER_SUNK_STATE);

  std::this_thread::sleep_for(DELAY_GENERAL_STEP);

  for (uint8_t i = 0; i < CELLS_PER_DAY; i++)
  {
    ctrl_pill_gate(PILL_GATE_WIDTH, PILL_GATE_OPEN_DIR, MOTOR_ENABLE);
    wait_for_pill_gate(MotorStatus::IDLE);

    std::this_thread::sleep_for(DELAY_GENERAL_STEP);
  }

  ctrl_pill_gate(PILL_GATE_WIDTH * NO_OF_PILL_GATES * PILL_GATE_CLOSE_MARGIN_FACTOR, PILL_GATE_CLOSE_DIR, MOTOR_ENABLE);
  wait_for_pill_gate(MotorStatus::IDLE);

  std::this_thread::sleep_for(DELAY_GENERAL_STEP);

  printer_.reset();
  printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial,
    printer_config_->port);
  RCLCPP_INFO(this->get_logger(), "printer initialized");
  init_printer_config();

  for (uint8_t i = 0; i < PKG_PREFIX; i++)
  {
    PackageInfo msg;
    msg.cn_name = "Init Pkg Mac";
    msg.en_name = "Init Pkg Mac";
    msg.date = "2024-11-30";
    msg.time = "17:00";
    msg.qr_code = "www.hkclr.hk";
    msg.drugs.push_back("DRUG 1");
    auto cmd = get_print_label_cmd(msg);
    printer_->runTask(cmd);
    RCLCPP_INFO(this->get_logger(), "printed a empty package");

    std::this_thread::sleep_for(DELAY_PKG_DIS_WAIT_PRINTER);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN_FACTOR, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis(MotorStatus::IDLE);

    std::this_thread::sleep_for(DELAY_PKG_DIS_BEFORE_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PUSH, MOTOR_ENABLE);
    wait_for_squeezer(MotorStatus::IDLE);

    std::this_thread::sleep_for(DELAY_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PULL, MOTOR_ENABLE);
    wait_for_squeezer(MotorStatus::IDLE);
  }

  printer_.reset();
  RCLCPP_INFO(this->get_logger(), "printer destroyed");

  ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_DISABLE);
  std::this_thread::sleep_for(DELAY_CONVEYOR_TESTING);
  ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_ENABLE);
  
  for (uint8_t i = 0; i < DAYS; i++)
  {
    ctrl_roller(1, 0, MOTOR_ENABLE);
    wait_for_roller(MotorStatus::IDLE);
  }
  
  ctrl_roller(0, 1, MOTOR_ENABLE);
  wait_for_roller(MotorStatus::IDLE);

  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  RCLCPP_INFO(this->get_logger(), "init_packaging_machine end");
}

void PackagingMachineNode::skip_order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PackagingOrder::Feedback>();
  auto& curr_order_status = feedback->curr_order_status;
  auto result = std::make_shared<PackagingOrder::Result>();

  goal_handle->publish_feedback(feedback);

  UnbindRequest msg;
  msg.packaging_machine_id = status_->packaging_machine_id;
  msg.order_id = goal->order_id;
  msg.material_box_id = goal->material_box_id;
  unbind_mtrl_box_publisher_->publish(msg);

  if (rclcpp::ok()) 
  {
    result->order_result = curr_order_status;
    goal_handle->succeed(result);
    
    status_->packaging_machine_state = PackagingMachineStatus::IDLE;
    
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
