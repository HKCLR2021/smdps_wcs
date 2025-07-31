#include "packaging_machine_control_system/packaging_machine_node.hpp"

// ===================================== Service =====================================
void PackagingMachineNode::init_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  if (status_->packaging_machine_state != PackagingMachineStatus::IDLE)
  {
    response->success = false;
    response->message = "State is not IDLE";
    return;
  }

  std::thread{std::bind(&PackagingMachineNode::init_packaging_machine, this)}.detach();
  
  response->success = true;
}

void PackagingMachineNode::heater_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (ctrl_heater(request->data))
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the heater";
  }
}

void PackagingMachineNode::cutter_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;

    if (request->data)
    {
      if (info_->cutter == 1)
      {
        response->success = false;
        response->message = "Cutter is in ON state";
        return;
      }
    }
    else
    {
      if (info_->cutter == 0)
      {
        response->success = false;
        response->message = "Stopper is in OFF state";
        return;
      }
    }
  }

  bool success = ctrl_cutter(request->data ? CUTTER_CLIP : CUTTER_RELEASE);

  if (success)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
    response->message = "Error to control the stopper";
  }
}

void PackagingMachineNode::stopper_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;

    if (request->data)
    {
      if (info_->stopper == STOPPER_SUNK_STATE)
      {
        response->success = false;
        response->message = "Stopper is in sunk state";
        return;
      }
    }
    else
    {
      if (info_->stopper == STOPPER_PROTRUDE_STATE)
      {
        response->success = false;
        response->message = "Stopper is in protrude state";
        return;
      }
    }
  }

  bool success = true;
  uint8_t MAX_RETIRES = 3;
  for (uint8_t i = 0; i < MAX_RETIRES; i++)
  {
    success &= ctrl_stopper(request->data ? STOPPER_PROTRUDE : STOPPER_SUNK);
    rclcpp::sleep_for(50ms);
  }

  if (success)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
    response->message = "Error to control the stopper";
  }
}

void PackagingMachineNode::mtrl_box_gate_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_)
      return;

    if (request->data)
    {
      if (info_->material_box_gate == MTRL_BOX_GATE_OPEN_STATE)
      {
        response->success = false;
        response->message = "Material Box Gate is in open state";
        return;
      }
    }
    else
    {
      if (info_->material_box_gate == MTRL_BOX_GATE_CLOSE_STATE)
      {
        response->success = false;
        response->message = "Material Box Gate is in close state";
        return;
      }
    }
  }

  if (ctrl_material_box_gate(request->data ? MTRL_BOX_GATE_OPEN : MTRL_BOX_GATE_CLOSE))
  {
    response->success = true;
  }
  else
  {
    response->success = false;
    response->message = "Error to control the material box gate";
  }
}

void PackagingMachineNode::conveyor_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;

    if (request->data)
    {
      if (motor_status_->con_state != MotorStatus::IDLE)
      {
        response->success = false;
        response->message = "Conveyor is not idle";
        return;
      }
    }
    else
    {
      if (motor_status_->con_state == MotorStatus::IDLE)
      {
        response->success = false;
        response->message = "Conveyor is already idle";
        return;
      }
    }
  }

  bool success = true;
  uint8_t MAX_RETIRES = 1;
  for (uint8_t i = 0; i < MAX_RETIRES; i++)
  {
    success &= ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, request->data);
    // rclcpp::sleep_for(50ms);
  }

  if (success)
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the conveyor";
  }
}

void PackagingMachineNode::pill_gate_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;
  }

  if (request->data)
  {
    if (ctrl_pill_gate(PILL_GATE_WIDTH, PILL_GATE_OPEN_DIR, MOTOR_ENABLE))
      response->success = true;
    else
    {
      response->success = false;
      response->message = "Error to control the Pill Gate";
    }
  }
  else
  {
    if (ctrl_pill_gate(PILL_GATE_WIDTH * NO_OF_PILL_GATES * PILL_GATE_CLOSE_MARGIN_FACTOR, PILL_GATE_CLOSE_DIR, MOTOR_ENABLE))
      response->success = true;
    else
    {
      response->success = false;
      response->message = "Error to control the Pill Gate";
    }
  }
}

void PackagingMachineNode::roller_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;
  }

  if (request->data)
  {
    if (ctrl_roller(1, 0, MOTOR_ENABLE))
      response->success = true;
    else
    {
      response->success = false;
      response->message = "Error to control the Roller";
    }
  }
  else
  {
    if (ctrl_roller(0, 1, MOTOR_ENABLE))
      response->success = true;
    else
    {
      response->success = false;
      response->message = "Error to control the Roller";
    }
  }
}

void PackagingMachineNode::squeezer_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;
  }

  ctrl_squeezer(SQUEEZER_ACTION_PUSH, MOTOR_ENABLE);
  wait_for_squeezer();

  rclcpp::sleep_for(DELAY_SQUEEZER);

  ctrl_squeezer(SQUEEZER_ACTION_PULL, MOTOR_ENABLE);
  wait_for_squeezer();

  response->success = true;
}

void PackagingMachineNode::pkg_len_handle(
  const std::shared_ptr<UInt8Srv::Request> request, 
  std::shared_ptr<UInt8Srv::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;
  }
  
  if (request->data == 1 && info_->pkg_len_level_2)
  {
    // 90mm -> 80mm
    // ctrl_pkg_len(1, MOTOR_ENABLE);
    // wait_for_pkg_len(MotorStatus::IDLE);
    
    // FIXME: 
    // ctrl_pkg_dis(50, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
    // wait_for_pkg_dis(MotorStatus::IDLE);
  }
  else if ((request->data == 2 && info_->pkg_len_level_1))
  {
    // 80mm -> 90mm
    // ctrl_pkg_len(2, MOTOR_ENABLE);
    // wait_for_pkg_len(MotorStatus::IDLE);

    // FIXME: required to print out pkg 
  }

  response->success = true;
}

void PackagingMachineNode::print_one_pkg_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;

    if (status_->packaging_machine_state != PackagingMachineStatus::IDLE)
    {
      response->success = false;
      response->message = "State is not IDLE";
      return;
    }
  }

  init_printer();
  init_printer_config();

  PackageInfo msg = create_printer_info_temp();
  auto cmd = get_print_label_cmd(msg);
  
  printer_->runTask(cmd);
  RCLCPP_INFO(this->get_logger(), "printed a empty package");

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
  
  printer_.reset();

  uint32_t remain_package = read_ribbon("package");
  uint32_t remain_thermal = read_ribbon("thermal");

  write_ribbon("package", remain_package - (status_->package_length));
  write_ribbon("thermal", remain_thermal - (status_->package_length));

  response->success = true;
}

void PackagingMachineNode::print_one_pkg_wo_squ_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_)
      return;

    if (status_->packaging_machine_state != PackagingMachineStatus::IDLE)
    {
      response->success = false;
      response->message = "State is not IDLE";
      return;
    }
  }

  init_printer();
  init_printer_config();

  PackageInfo msg = create_printer_info_temp();
  auto cmd = get_print_label_cmd(msg);
  
  printer_->runTask(cmd);
  RCLCPP_INFO(this->get_logger(), "printed a empty package");

  rclcpp::sleep_for(DELAY_PKG_DIS_WAIT_PRINTER);
  ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN_FACTOR, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
  wait_for_pkg_dis();

  ctrl_pkg_dis(PKG_DIS_UNFEED_LEN, PKG_DIS_UNFEED_DIR, MOTOR_ENABLE);
  wait_for_pkg_dis();

  printer_.reset();

  uint32_t remain_package = read_ribbon("package");
  uint32_t remain_thermal = read_ribbon("thermal");

  write_ribbon("package", remain_package - (status_->package_length));
  write_ribbon("thermal", remain_thermal - (status_->package_length));

  response->success = true;
}

// This service is designed for debugging only
// It should not be used in normal case
void PackagingMachineNode::state_ctrl_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (request->data)
    status_->packaging_machine_state = PackagingMachineStatus::BUSY;
  else
    status_->packaging_machine_state = PackagingMachineStatus::IDLE;
  
  response->success = true;
}

// This service is designed for testing only
// It should not be used in normal case
void PackagingMachineNode::skip_pkg_ctrl_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_->skip_packaging = request->data;
  response->success = true;
}

// This service is designed for testing only
// It should not be used in normal case
void PackagingMachineNode::enable_heater_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  ctrl_heater(HEATER_OFF);
  response->success = true;

  std::lock_guard<std::mutex> lock(mutex_);
  enable_heater_ = request->data;
}

void PackagingMachineNode::update_package_handle(
  const std::shared_ptr<UInt32Srv::Request> request, 
  std::shared_ptr<UInt32Srv::Response> response)
{
  write_ribbon("package", request->data);
  response->success = true;
}

void PackagingMachineNode::update_thermal_handle(
  const std::shared_ptr<UInt32Srv::Request> request, 
  std::shared_ptr<UInt32Srv::Response> response)
{
  write_ribbon("thermal", request->data);
  response->success = true;
}