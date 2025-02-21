#include "packaging_machine_control_system/packaging_machine_node.hpp"

// ===================================== heater =====================================
bool PackagingMachineNode::ctrl_heater(const bool on)
{
  bool success = write_heater(on ? 1 : 0);
  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the heater", on ? "turn on" : "turn off");
  return success;
}

bool PackagingMachineNode::write_heater(const uint32_t data)
{
  return call_co_write(0x6003, 0x0, data);
}

bool PackagingMachineNode::read_heater(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6003, 0x0, data);
}

// ===================================== stopper =====================================
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
  return call_co_read(0x6054, 0x0, data);
}

// ===================================== material_box_gate =====================================
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
  return call_co_read(0x6055, 0x0, data);
}

// ===================================== cutter =====================================
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
  return call_co_read(0x6056, 0x0, data);
}

// ===================================== pkg_dis =====================================
bool PackagingMachineNode::ctrl_pkg_dis(
  const float length, 
  const bool feed, 
  const bool ctrl
)
{
  bool success = true;

  success &= call_co_write(0x6011, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PKG_DIS_RADIUS)));
  success &= call_co_write(0x6012, 0x0, feed ? 1 : 0); // Set to 0 to feed the package out
  success &= call_co_write(0x6019, 0x0, ctrl ? 1 : 0);

  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the package: %.2fmm", feed ? "feed" : "unfeed", length);
  return success;
}

bool PackagingMachineNode::read_pkg_dis_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6018, 0x0, data);
}

bool PackagingMachineNode::read_pkg_dis_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6019, 0x0, data);
}

// ===================================== pill_gate =====================================
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
  return call_co_read(0x6028, 0x0, data);
}

bool PackagingMachineNode::read_pill_gate_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6029, 0x0, data);
}

// ===================================== squeezer =====================================
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
  // std::this_thread::sleep_for(DELAY_CO_L);

  success &= call_co_write(0x6079, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "%s the squeezer", squeeze ? "push" : "pull");

  return success;
}

bool PackagingMachineNode::read_squeezer_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6078, 0x0, data);
}

bool PackagingMachineNode::read_squeezer_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6079, 0x0, data);
}

// ===================================== conveyor =====================================
bool PackagingMachineNode::ctrl_conveyor(
  const uint16_t speed, 
  const bool stop_by_ph, 
  const bool fwd, 
  const bool ctrl)
{
  bool success = true;

  if (!ctrl) 
  {
    success &= call_co_write(0x6089, 0x0, 0);
    if (success)
      RCLCPP_INFO(this->get_logger(), "Stop the conveyor");

    return success;
  }

  success &= call_co_write(0x6080, 0x0, speed > 3000 ? 3000 : speed);
  success &= call_co_write(0x6081, 0x0, stop_by_ph ? 1 : 0);
  success &= call_co_write(0x6082, 0x0, fwd ? 0 : 1);
  success &= call_co_write(0x6089, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "moving the conveyor %s", stop_by_ph ? "with stop by photoelectric sensor" : "");
  
  return success;
}

bool PackagingMachineNode::read_conveyor_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6088, 0x0, data);
}

bool PackagingMachineNode::read_conveyor_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6089, 0x0, data);
}

// ===================================== roller =====================================
bool PackagingMachineNode::ctrl_roller(
  const uint8_t days, 
  const bool home, 
  const bool ctrl)
{
  bool success = true;

  if (!ctrl) 
  {
    success &= call_co_write(0x6039, 0x0, 0);
    if (success)
      RCLCPP_INFO(this->get_logger(), "Stop the roller");

    return success;
  }

  if (home) 
  {
    success &= call_co_write(0x6030, 0x0, 1); // must be 1 step
    success &= call_co_write(0x6037, 0x0, 1); // set mode 1 to go home
  }  else {
    success &= call_co_write(0x6030, 0x0, days > DAYS ? DAYS : days);
    success &= call_co_write(0x6037, 0x0, 0); // set mode 0 to go X day(s)
  }

  success &= call_co_write(0x6032, 0x0, 0); // direction must be 0 
  success &= call_co_write(0x6039, 0x0, 1);

  if (success)
  {
    if (home)
      RCLCPP_INFO(this->get_logger(), "moving the roller to home");
    else
      RCLCPP_INFO(this->get_logger(), "moving the roller to %s day(s)", std::to_string(days).c_str());
  }

  return success;
}

bool PackagingMachineNode::read_roller_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6038, 0x0, data);
}

bool PackagingMachineNode::read_roller_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6039, 0x0, data);
}

// ===================================== pkg_len =====================================
bool PackagingMachineNode::ctrl_pkg_len(
  const uint8_t level, 
  const bool ctrl)
{
  bool success = true;
  if (!ctrl) 
  {
    success &= call_co_write(0x6049, 0x0, 0);
    return success;
  }

  success &= call_co_write(0x6040, 0x0, 1); // move 1 step

  switch (level)
  {
  case 1:
    success &= call_co_write(0x6042, 0x0, 0); // moving downward
    break;
  case 2:
    success &= call_co_write(0x6042, 0x0, 1); // moving upward
    break;
  default:
    return ctrl_pkg_len(0, 0);
    break;
  }

  success &= call_co_write(0x6049, 0x0, 1);

  if (success)
    RCLCPP_INFO(this->get_logger(), "moving the pkg len to level ???"); // FIXME
  
  return success;
}

bool PackagingMachineNode::read_pkg_len_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6048, 0x0, data);
}

bool PackagingMachineNode::read_pkg_len_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6049, 0x0, data);
}

// ===================================== printer =====================================
void PackagingMachineNode::init_printer_config()
{
  printer_->configure(printer_config_->endpoint_in, printer_config_->endpoint_out, printer_config_->timeout);
  printer_->addDefaultConfig("SIZE", "75 mm,80 mm");
  printer_->addDefaultConfig("GAP", "0 mm, 0mm");
  printer_->addDefaultConfig("SPEED", "1");
  printer_->addDefaultConfig("DENSITY", "10");
  printer_->addDefaultConfig("DIRECTION", "0, 0");
  printer_->addDefaultConfig("REFERENCE", "-90, -120"); // FIXME
  printer_->addDefaultConfig("OFFSET", "0 mm");
  printer_->addDefaultConfig("SHIFT", "0");
  printer_->addDefaultConfig("SET", "TEAR OFF");
  printer_->addDefaultConfig("SET", "REWIND OFF");
  printer_->addDefaultConfig("SET", "PEEL OFF");
  printer_->addDefaultConfig("SET", "CUTTER OFF");
  printer_->addDefaultConfig("SET", "PARTIAL_CUTTER OFF");
  printer_->addDefaultConfig("CLS");
}

std::vector<std::string> PackagingMachineNode::get_print_label_cmd(PackageInfo msg)
{
  std::vector<std::string> cmds{};
  if (!msg.en_name.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Add a english name: %s", msg.en_name.c_str());

    std::string gbk_cn = printer_->convert_utf8_to_gbk(msg.cn_name);
    std::string cn = "TEXT 240,180,\"TSS24.BF2\",0,2,2,\"" + gbk_cn + "\"";
    cmds.emplace_back(cn);
    std::string en = "TEXT 600,186,\"TSS24.BF2\",0,2,2,\"" + msg.en_name + "\"";
    cmds.emplace_back(en);
    std::string d = "TEXT 240,270,\"4\",0,1,1,\"" + msg.date + "\"";
    cmds.emplace_back(d);
    std::string t = "TEXT 240,334,\"4\",0,1,1,\"" + msg.time + "\"";
    cmds.emplace_back(t);
    cmds.emplace_back("QRCODE 684,252,L,6,A,0,\"" + msg.qr_code + "\"");
    for (size_t index = 0; index < msg.drugs.size(); ++index) 
    {
      std::string utf_md = msg.drugs[index];
      std::string gbk_md = printer_->convert_utf8_to_gbk(utf_md);
      int y = 400 + index * 64;
      std::string y_label = std::to_string(y);
      std::string m = "TEXT 240,"+ y_label + ",\"TSS24.BF2\",0,2,2,\"" + gbk_md + "\"";
      cmds.emplace_back(m);
    }
  }

  cmds.emplace_back("PRINT 1,1");
  RCLCPP_DEBUG(this->get_logger(), "printer commands are ready");
  return cmds;
}

// ===================================== wait for =====================================
void PackagingMachineNode::wait_for_stopper(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_stopper(data);
    info_->stopper = *data;
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
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_material_box_gate(data);
    info_->material_box_gate = *data;
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
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_cutter(data);
    info_->cutter = *data;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR);

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pkg_dis_state(state);
    std::this_thread::sleep_for(DELAY_CO);
    read_pkg_dis_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO);
    RCLCPP_DEBUG(this->get_logger(), "pkg_dis_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->pkg_dis_state = *state;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pill_gate_state(state);
    std::this_thread::sleep_for(DELAY_CO);
    read_pill_gate_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO);
    RCLCPP_DEBUG(this->get_logger(), "pill_gate_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->pill_gate_state = *state;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    std::this_thread::sleep_for(DELAY_CO_L);
    read_squeezer_state(state);
    std::this_thread::sleep_for(DELAY_CO_L);
    read_squeezer_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO_L);
    RCLCPP_DEBUG(this->get_logger(), "squeezer_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->squ_state = *state;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    std::this_thread::sleep_for(DELAY_CO_L);
    read_conveyor_state(state);
    std::this_thread::sleep_for(DELAY_CO_L);
    read_conveyor_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO_L);
    RCLCPP_DEBUG(this->get_logger(), "conveyor_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->con_state = *state;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_roller_state(state);
    std::this_thread::sleep_for(DELAY_CO);
    read_roller_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO);
    RCLCPP_DEBUG(this->get_logger(), "roller_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->roller_state = *state;
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
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pkg_len_state(state);
    std::this_thread::sleep_for(DELAY_CO);
    read_pkg_len_ctrl(ctrl);
    std::this_thread::sleep_for(DELAY_CO);
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_state: %d, ctrl: %d", *state, *ctrl);

    motor_status_->pkg_len_state = *state;
    bool termination = *state == target_state && *ctrl == 0;

    if (termination)
    {
      RCLCPP_INFO(this->get_logger(), "pkg_len_state is idle");
      break;
    }
    rate.sleep();
  }
}