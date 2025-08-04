#include "packaging_machine_control_system/packaging_machine_node.hpp"

// ===================================== wait for =====================================
void PackagingMachineNode::wait_for_stopper(const uint32_t stop_condition)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);
  
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);

    if (!read_stopper(data))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read data failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    // info_->stopper = *data;
    RCLCPP_DEBUG(this->get_logger(), "stopper: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the stopper. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_material_box_gate(const uint32_t stop_condition)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);
  
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);

    if (!read_material_box_gate(data))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read data failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    // info_->material_box_gate = *data;
    RCLCPP_DEBUG(this->get_logger(), "material_box_gate: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the material_box_gate. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_cutter(const uint32_t stop_condition)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);

  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);

    if (!read_cutter(data))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read data failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    // info_->cutter = *data;
    RCLCPP_DEBUG(this->get_logger(), "cutter: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the cutter. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pkg_dis(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);
  
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    
    if (!read_pkg_dis_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    if (!read_pkg_dis_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    RCLCPP_DEBUG(this->get_logger(), "pkg_dis_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->pkg_dis_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "pkg_dis is idle");
      break;
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pill_gate(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);
  
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);

    if (!read_pill_gate_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    if (!read_pill_gate_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "pill_gate_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->pill_gate_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "pill_gate is idle");
      break;
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_squeezer(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);

  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);

    if (!read_squeezer_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    if (!read_squeezer_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    RCLCPP_DEBUG(this->get_logger(), "squeezer_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->squ_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "squeezer_state is idle");
      break;
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_conveyor(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);
  
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    
    if (!read_conveyor_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    if (!read_conveyor_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "conveyor_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->con_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "conveyor_state is idle");
      break;
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_roller(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);

  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    
    if (!read_roller_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }

    if (!read_roller_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "roller_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->roller_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "roller_state is idle");
      break;
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pkg_len(const uint8_t target_state)
{
  rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
  rclcpp::Rate rate(2);

  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    
    if (!read_pkg_len_state(state))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read state failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    if (!read_pkg_len_ctrl(ctrl))
    {
      RCLCPP_WARN(this->get_logger(), "%s, read ctrl failed", __FUNCTION__);
      rate.sleep();
      continue;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_state: %d, ctrl: %d", *state, *ctrl);

    // motor_status_->pkg_len_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "pkg_len_state is idle");
      break;
    }

    rate.sleep();
  }
}

// template<typename StateReader, typename CtrlReader>
// void PackagingMachineNode::wait_for_motor_state(
//   StateReader read_state_func,
//   CtrlReader read_ctrl_func,
//   const uint8_t target_state,
//   const std::string& motor_name)
// {
//   rclcpp::sleep_for(DELAY_MOTOR_WAIT_FOR);
//   rclcpp::Rate rate(2);

//   while (rclcpp::ok())
//   {
//     std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
//     std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    
//     if (!read_state_func(state))
//     {
//       RCLCPP_WARN(this->get_logger(), "read %s state failed", motor_name.c_str());
//       rate.sleep();
//       continue;
//     }

//     if (!read_ctrl_func(ctrl))
//     {
//       RCLCPP_WARN(this->get_logger(), "read %s ctrl failed", motor_name.c_str());
//       rate.sleep();
//       continue;
//     }
    
//     RCLCPP_DEBUG(this->get_logger(), "%s_state: %d, ctrl: %d", 
//                  motor_name.c_str(), *state, *ctrl);

//     bool termination = *state == target_state && *ctrl == 0;

//     if (termination)
//     {
//       RCLCPP_INFO(this->get_logger(), "%s_state is idle", motor_name.c_str());
//       break;
//     }

//     rate.sleep();
//   }
// }