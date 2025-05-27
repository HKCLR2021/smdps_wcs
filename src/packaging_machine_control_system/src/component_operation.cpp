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
    success &= call_co_write(0x6042, 0x0, 1); // moving upward
    break;
  case 2:
    success &= call_co_write(0x6042, 0x0, 0); // moving downward
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

void PackagingMachineNode::init_printer(void)
{
  printer_.reset();
  printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial,
    printer_config_->port);

  RCLCPP_INFO(this->get_logger(), "printer initialized");
}

void PackagingMachineNode::init_printer_config(void)
{
  printer_->configure(printer_config_->endpoint_in, printer_config_->endpoint_out, printer_config_->timeout);

  std::string size_str = "75 mm," + std::to_string(status_->package_length) + " mm";
  
  printer_->addDefaultConfig("SIZE", size_str);
  printer_->addDefaultConfig("GAP", "0 mm, 0 mm");
  printer_->addDefaultConfig("SPEED", "1");
  printer_->addDefaultConfig("DENSITY", "15");
  printer_->addDefaultConfig("DIRECTION", "0,0");
  printer_->addDefaultConfig("REFERENCE", "0,0");
  printer_->addDefaultConfig("OFFSET", "0 mm");
  printer_->addDefaultConfig("SHIFT", "-177"); // 15 mm
  printer_->addDefaultConfig("SET", "TEAR OFF");
  printer_->addDefaultConfig("SET", "REWIND OFF");
  printer_->addDefaultConfig("SET", "PEEL OFF");
  printer_->addDefaultConfig("SET", "CUTTER OFF");
  printer_->addDefaultConfig("SET", "PARTIAL_CUTTER OFF");
  printer_->addDefaultConfig("CLS");
}

std::string PackagingMachineNode::add_drug_space(std::string drug_str)
{
  for_each(drug_str.begin(), drug_str.end(), [](char& c) {
      c = toupper(c);
  });

  if (drug_str.length() > MAX_DRUG_LEN) 
  {
    drug_str = drug_str.substr(0, MAX_DRUG_LEN);
    return drug_str;
  }

  uint8_t num_of_space = 0;
  while (drug_str.length() < MAX_DRUG_LEN)
  {
    drug_str.insert(drug_str.end() - 1, ' ');
    num_of_space++;
  }
  RCLCPP_INFO(this->get_logger(), "Added %d number of space to drug name", num_of_space);

  // if (drug_str.find("TABLET") != std::string::npos) 
  // {
  //   drug_str.append(" TAB");
  //   RCLCPP_INFO(this->get_logger(), "Added TAB unit");
  // } 
  // else if (drug_str.find("CAPSULE") != std::string::npos) 
  // {
  //   drug_str.append(" CAP");
  //   RCLCPP_INFO(this->get_logger(), "Added CAP unit");
  // } 
  // else
  // {
  //   drug_str.append(" EA");
  //   RCLCPP_INFO(this->get_logger(), "Added EA unit");
  // }

  return drug_str;
}

smdps_msgs::msg::PackageInfo PackagingMachineNode::create_printer_info_temp(void)
{
  PackageInfo msg;

  msg.cn_name = "Centre A";
  msg.en_name = "Sam";

  if (printer_test_date_index < printer_test_date.size())
    msg.date = printer_test_date[printer_test_date_index];
  else
    msg.date = "out of range";

  if (printer_test_meal_index < printer_test_meal.size())
    msg.time = printer_test_meal[printer_test_meal_index++];
  else
    msg.time = "out of range";
  
  msg.qr_code = "https://www.hkclr.hk";

  std::string drug;
  drug = "PARACETAMOL TABLET 500MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "HYDRALAZINE HCI TABLET 25MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "FAMOTIDINE TABLET 20MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "COLCINA TABLET 0.5MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "DIMETHYLPOLYSILOXANE TABLET 40MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "SENNA TABLET 7.5MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  // msg.drugs.push_back("12345678901234567890123456789012345678901234567890");

  RCLCPP_INFO(this->get_logger(), add_drug_space(drug).c_str());


  if (printer_test_meal_index >= printer_test_meal.size())
  {
    printer_test_meal_index = 0; 
    printer_test_date_index++;
  }
  
  if (printer_test_date_index >= printer_test_date.size())
    printer_test_date_index = 0;

  return msg;
}

std::vector<std::string> PackagingMachineNode::get_print_label_cmd(PackageInfo msg)
{
  std::vector<std::string> cmds{};
  if (!msg.en_name.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Add a english name: %s", msg.en_name.c_str());

    // std::string gbk_cn = printer_->convert_utf8_to_gbk(msg.cn_name);
    // std::string cn = "TEXT 240,180,\"3\",0,2,2,\"" + gbk_cn + "\"";
    std::string cn = "TEXT 150,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.cn_name + "\"";
    cmds.emplace_back(cn);
    // std::string en = "TEXT 150,225,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.en_name + "\"";
    std::string en = "TEXT 190,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.en_name + "\"";
    cmds.emplace_back(en);
    // std::string d = "TEXT 200,575,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.date + "\"";
    std::string d = "TEXT 230,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.date + "\"";
    cmds.emplace_back(d);
    // std::string t = "TEXT 250,575,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.time + "\"";
    std::string t = "TEXT 270,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.time + "\"";
    cmds.emplace_back(t);
    // std::string q = "QRCODE 200,225,H,8,A,270,\"" + msg.qr_code + "\"";
    std::string q = "QRCODE 150,275,H,9,A,270,\"" + msg.qr_code + "\"";
    cmds.emplace_back(q);
    for (size_t index = 0; index < msg.drugs.size(); index++) 
    {
      std::string utf_md = msg.drugs[index];
      // std::string gbk_md = printer_->convert_utf8_to_gbk(utf_md);
      int x = 400 + index * 40;
      std::string x_label = std::to_string(x);
      std::string m = "TEXT " + x_label + ",550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + utf_md + "\"";
      cmds.emplace_back(m);
    }
  }

  cmds.emplace_back("PRINT 1,1");
  // cmds.emplace_back("EOP");
  RCLCPP_DEBUG(this->get_logger(), "printer commands are ready");
  return cmds;
}

// ===================================== wait for =====================================
void PackagingMachineNode::wait_for_stopper(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  // rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_stopper(data);
    std::this_thread::sleep_for(DELAY_CO);
    info_->stopper = *data;
    RCLCPP_DEBUG(this->get_logger(), "stopper: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the stopper. Exiting");
      break; 
    }

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_material_box_gate(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  // rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_material_box_gate(data);
    std::this_thread::sleep_for(DELAY_CO);
    info_->material_box_gate = *data;
    RCLCPP_DEBUG(this->get_logger(), "material_box_gate: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the material_box_gate. Exiting");
      break; 
    }

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_cutter(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(DELAY_VALVE_WAIT_FOR); 

  // rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_cutter(data);
    std::this_thread::sleep_for(DELAY_CO);
    info_->cutter = *data;
    RCLCPP_DEBUG(this->get_logger(), "cutter: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the cutter. Exiting");
      break; 
    }

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_pkg_dis(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR);

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_pill_gate(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_squeezer(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_conveyor(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_roller(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}

void PackagingMachineNode::wait_for_pkg_len(const uint8_t target_state)
{
  std::this_thread::sleep_for(DELAY_MOTOR_WAIT_FOR); 

  // rclcpp::Rate rate(2);
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

    // rate.sleep();
    std::this_thread::sleep_for(500ms); 
  }
}