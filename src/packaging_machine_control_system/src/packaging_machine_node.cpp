#include "packaging_machine_control_system/packaging_machine_node.hpp"

PackagingMachineNode::PackagingMachineNode(const rclcpp::NodeOptions& options)
: Node("packaging_machine_node", options)
{
  status_ = std::make_shared<PackagingMachineStatus>();
  motor_status_ = std::make_shared<MotorStatus>();
  info_ = std::make_shared<PackagingMachineInfo>();
  printer_config_ = std::make_shared<Config>();

  this->declare_parameter<uint8_t>("packaging_machine_id", 0);
  this->declare_parameter<std::vector<long int>>("default_states", std::vector<long int>{});
  this->declare_parameter<std::vector<long int>>("ports", std::vector<long int>{});
  this->declare_parameter<bool>("simulation", false);

  this->get_parameter("packaging_machine_id", status_->packaging_machine_id);
  this->get_parameter("simulation", sim_);

  // for testing only
  printer_test_date.push_back("2024-03-31");
  printer_test_date.push_back("2024-04-01");
  printer_test_date.push_back("2024-04-02");
  printer_test_meal.push_back("Morning");
  printer_test_meal.push_back("Noon");
  printer_test_meal.push_back("Afternoon");
  printer_test_meal.push_back("Evening");

  printer_font_ = 3;
  enable_heater_ = true;

  std::vector<long int> default_states = this->get_parameter("default_states").as_integer_array();
  status_->packaging_machine_state = default_states[status_->packaging_machine_id - 1];

  std::vector<long int> ports = this->get_parameter("ports").as_integer_array();
  printer_config_->port = ports[status_->packaging_machine_id - 1];

  RCLCPP_INFO(this->get_logger(), "ID: %d", status_->packaging_machine_id);
  RCLCPP_INFO(this->get_logger(), "default_states size: %ld", default_states.size());
  RCLCPP_INFO(this->get_logger(), "packaging_machine_state: %d", status_->packaging_machine_state);
  RCLCPP_INFO(this->get_logger(), "port: %d", printer_config_->port);

  motor_status_->id = status_->packaging_machine_id;
  info_->id = status_->packaging_machine_id;
  
  this->declare_parameter<uint16_t>("vendor_id", 0);
  this->declare_parameter<uint16_t>("product_id", 0);
  this->declare_parameter<uint8_t>("bus_number", 0);
  this->declare_parameter<uint8_t>("device_number", 0);
  this->declare_parameter<std::string>("serial", "");
  this->declare_parameter<uint8_t>("endpoint_in", 0);
  this->declare_parameter<uint8_t>("endpoint_out", 0);
  this->declare_parameter<int>("timeout", 0);
  this->declare_parameter<uint8_t>("dots_per_mm", 0);
  this->declare_parameter<uint8_t>("direction", 0);
  this->declare_parameter<int>("total", 0);
  this->declare_parameter<int>("interval", 0);
  this->declare_parameter<bool>("offset_x", false);
  this->declare_parameter<bool>("offset_y", false);

  this->get_parameter("vendor_id", printer_config_->vendor_id);
  this->get_parameter("product_id", printer_config_->product_id);
  this->get_parameter("bus_number", printer_config_->bus_number);
  this->get_parameter("device_number", printer_config_->device_number);
  // this->get_parameter("port", printer_config_->port);
  this->get_parameter("serial", printer_config_->serial);
  this->get_parameter("endpoint_in", printer_config_->endpoint_in);
  this->get_parameter("endpoint_out", printer_config_->endpoint_out);
  this->get_parameter("timeout", printer_config_->timeout);
  this->get_parameter("dots_per_mm", printer_config_->dots_per_mm);
  this->get_parameter("direction", printer_config_->direction);
  this->get_parameter("total", printer_config_->total);
  this->get_parameter("interval", printer_config_->interval);
  this->get_parameter("offset_x", printer_config_->offset_x);
  this->get_parameter("offset_y", printer_config_->offset_y);

  status_->header.frame_id = "Packaging Machine";
  status_->conveyor_state = PackagingMachineStatus::AVAILABLE;
  status_->canopen_state = PackagingMachineStatus::NORMAL;
  status_->package_length = 80; // FIXME

  co_cli_read_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  co_cli_write_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_conveyor_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_stopper_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  status_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  normal_timer_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;

  status_timer_ = this->create_wall_timer(1s, std::bind(&PackagingMachineNode::pub_status_cb, this), status_cbg_);
  heater_timer_ = this->create_wall_timer(5s, std::bind(&PackagingMachineNode::heater_cb, this), normal_timer_cbg_);
  con_state_timer_ = this->create_wall_timer(100ms, std::bind(&PackagingMachineNode::con_state_cb, this), status_cbg_);
  remain_length_timer_ = this->create_wall_timer(500ms, std::bind(&PackagingMachineNode::remain_length_cb, this), normal_timer_cbg_);
  once_timer_ = this->create_wall_timer(3s, std::bind(&PackagingMachineNode::init_timer, this), normal_timer_cbg_);

  // add a "/" prefix to topic name avoid adding a namespace
  status_publisher_ = this->create_publisher<PackagingMachineStatus>("/packaging_machine_status", 10); 
  motor_status_publisher_ = this->create_publisher<MotorStatus>("/motor_status", 10); 
  info_publisher_ = this->create_publisher<PackagingMachineInfo>("/info", 10); 
  unbind_mtrl_box_publisher_ = this->create_publisher<UnbindRequest>("unbind_material_box_id", 10); 
  skip_pkg_publisher_ = this->create_publisher<Bool>("skip_packaging", 10); 

  tpdo_pub_ = this->create_publisher<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/tpdo", 
    10);

  rpdo_sub_ = this->create_subscription<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/rpdo", 
    10,
    std::bind(&PackagingMachineNode::rpdo_cb, this, _1),
    rpdo_options);
  
  co_read_client_ = this->create_client<CORead>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/sdo_read",
    rmw_qos_profile_services_default,
    co_cli_read_cbg_);

  co_write_client_ = this->create_client<COWrite>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/sdo_write",
    rmw_qos_profile_services_default,
    co_cli_write_cbg_);

  init_pkg_mac_service_ = this->create_service<Trigger>(
    "init_package_machine", 
    std::bind(&PackagingMachineNode::init_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  heater_service_ = this->create_service<SetBool>(
    "heater_operation", 
    std::bind(&PackagingMachineNode::heater_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  cutter_service_ = this->create_service<SetBool>(
    "cutter_operation", 
    std::bind(&PackagingMachineNode::cutter_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  stopper_service_ = this->create_service<SetBool>(
    "stopper_operation", 
    std::bind(&PackagingMachineNode::stopper_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_stopper_ser_cbg_);

  mtrl_box_gate_service_ = this->create_service<SetBool>(
    "material_box_gate_operation", 
    std::bind(&PackagingMachineNode::mtrl_box_gate_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  conveyor_service_ = this->create_service<SetBool>(
    "conveyor_operation", 
    std::bind(&PackagingMachineNode::conveyor_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_conveyor_ser_cbg_);

  pill_gate_service_ = this->create_service<SetBool>(
    "pill_gate_operation", 
    std::bind(&PackagingMachineNode::pill_gate_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  roller_service_ = this->create_service<SetBool>(
    "roller_operation", 
    std::bind(&PackagingMachineNode::roller_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  squeezer_service_ = this->create_service<Trigger>(
    "squeezer_operation", 
    std::bind(&PackagingMachineNode::squeezer_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  pkg_len_service_ = this->create_service<UInt8Srv>(
    "package_length_operation", 
    std::bind(&PackagingMachineNode::pkg_len_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  print_one_pkg_service_ = this->create_service<Trigger>(
    "print_one_package", 
    std::bind(&PackagingMachineNode::print_one_pkg_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  print_one_pkg_wo_squ_service_ = this->create_service<Trigger>(
    "print_one_package_wo_squeeze", 
    std::bind(&PackagingMachineNode::print_one_pkg_wo_squ_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  state_ctrl_service_ = this->create_service<SetBool>(
    "state_control", 
    std::bind(&PackagingMachineNode::state_ctrl_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  skip_pkg_service_ = this->create_service<SetBool>(
    "skip_packaging_control", 
    std::bind(&PackagingMachineNode::skip_pkg_ctrl_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  enable_heater_service_ = this->create_service<SetBool>(
    "enable_heater", 
    std::bind(&PackagingMachineNode::enable_heater_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);
  
  update_package_service_ = this->create_service<UInt32Srv>(
    "update_remain_package_length", 
    std::bind(&PackagingMachineNode::update_package_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  update_thermal_service_ = this->create_service<UInt32Srv>(
    "update_remain_thermal_length", 
    std::bind(&PackagingMachineNode::update_thermal_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  this->action_server_ = rclcpp_action::create_server<PackagingOrder>(
    this,
    "packaging_order",
    std::bind(&PackagingMachineNode::handle_goal, this, _1, _2),
    std::bind(&PackagingMachineNode::handle_cancel, this, _1),
    std::bind(&PackagingMachineNode::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  RCLCPP_INFO(this->get_logger(), "Packaging Machine Node %d is up.", status_->packaging_machine_id);

  if (!sim_)
  {
    co_read_wait_for_service();
    co_write_wait_for_service();
    
    RCLCPP_INFO(this->get_logger(), "The CO Service client is up.");
  }
  
  // for testing printer only
  // for (uint8_t i = 0; i < 10; i++)
  // {
  //   init_printer();
  //   init_printer_config();

  //   PackageInfo msg = create_printer_info_temp();
  //   auto cmd = get_print_label_cmd(msg);
    
  //   printer_->runTask(cmd);
  //   RCLCPP_INFO(this->get_logger(), "printed a empty package");
  //   printer_font_++;
  // }
}

void PackagingMachineNode::pub_status_cb(void)
{
  Bool skip_pkg_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    skip_pkg_msg.data = status_->skip_packaging;
    status_->header.stamp = this->get_clock()->now();
  }

  status_publisher_->publish(*status_);
  motor_status_publisher_->publish(*motor_status_);
  info_publisher_->publish(*info_);
  skip_pkg_publisher_->publish(skip_pkg_msg);
}

void PackagingMachineNode::heater_cb(void)
{
  // std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  // lock.lock();

  // if (enable_heater_ && info_->temperature < MIN_TEMP)
  if (enable_heater_)
  {
    if (info_->temperature < MIN_TEMP)
      RCLCPP_INFO(this->get_logger(), "Current heater temperature: %d [less than 100 Celsius]", info_->temperature);
    
    // lock.unlock();

    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    bool success = read_heater(state);

    if (success && *state == HEATER_OFF_STATE)
    {
      ctrl_heater(HEATER_ON);
      RCLCPP_INFO(this->get_logger(), "Turn on in the timer callback");
    }
  }
}

void PackagingMachineNode::con_state_cb(void)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (motor_status_->con_state == MotorStatus::RUNNING && info_->stopper == STOPPER_SUNK)
  {
    status_->conveyor_state = PackagingMachineStatus::AVAILABLE;
    RCLCPP_DEBUG(this->get_logger(), "set conveyor_state to AVAILABLE");
  }
  else
  {
    status_->conveyor_state = PackagingMachineStatus::UNAVAILABLE;
    RCLCPP_DEBUG(this->get_logger(), "set conveyor_state to UNAVAILABLE");
  }
}

void PackagingMachineNode::remain_length_cb(void)
{
  uint32_t remain_package = read_ribbon("package");
  uint32_t remain_thermal = read_ribbon("thermal");

  std::lock_guard<std::mutex> lock(mutex_);
  status_->remain_package_length = remain_package;
  status_->remain_thermal_length = remain_thermal;
}

void PackagingMachineNode::init_timer(void)
{
  RCLCPP_INFO(this->get_logger(), "Init timer start");
  bool success = true;
  success &= ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_ENABLE);

  success &= ctrl_stopper(STOPPER_SUNK);
  wait_for_stopper(STOPPER_SUNK_STATE);

  if (success)
  {
    is_initialized_ = true;
    once_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Init timer cancelled!");
  }
}

void PackagingMachineNode::rpdo_cb(const COData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  switch (msg->index)
  {
  case 0x6001:
    info_->temperature = static_cast<uint8_t>(msg->data);
    break;
  case 0x6008:
    info_->temperature_ctrl = static_cast<uint16_t>(msg->data);
    break;
  case 0x6018:
    motor_status_->pkg_dis_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6026:
    motor_status_->pill_gate_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6028:
    motor_status_->pill_gate_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6038:
    motor_status_->roller_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6046:
    motor_status_->pkg_len_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6048:
    motor_status_->pkg_len_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6058: {
    uint8_t input = static_cast<uint8_t>(msg->data);
    info_->stopper           = input        & 0x1 ? STOPPER_PROTRUDE_STATE : STOPPER_SUNK_STATE;
    info_->material_box_gate = (input >> 1) & 0x1 ? MTRL_BOX_GATE_OPEN_STATE : MTRL_BOX_GATE_CLOSE_STATE ;
    info_->cutter            = (input >> 2) & 0x1; // FIXME
    RCLCPP_DEBUG(this->get_logger(), "stopper: %s", info_->stopper ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "material_box_gate: %s", info_->material_box_gate ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "cutter: %s", info_->cutter ? "1" : "0");
    break;
  }
  case 0x6068: {
    uint8_t input = static_cast<uint8_t>(msg->data);
    for (int i = NO_OF_REED_SWITCHS - 1; i >= 0; i--) 
    {
      info_->rs_state[i] = (input & 1);
      input >>= 1;
    }
    break;
  }
  case 0x6076:
    motor_status_->squ_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6078:
    motor_status_->squ_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6088:
    motor_status_->con_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6090: {
    uint8_t input = static_cast<uint16_t>(msg->data);
    info_->conveyor        = input & 0x1;
    info_->squeeze         = (input >> 1) & 0x1;
    info_->squeeze_home    = (input >> 2) & 0x1;
    info_->roller_step     = (input >> 3) & 0x1;
    info_->roller_home     = (input >> 4) & 0x1;
    info_->pill_gate_home  = (input >> 5) & 0x1;
    info_->pkg_len_level_1 = (input >> 6) & 0x1;
    info_->pkg_len_level_2 = (input >> 7) & 0x1;

    if (info_->pkg_len_level_1)
      status_->package_length = 90;
    else if (info_->pkg_len_level_2)
      status_->package_length = 80;

    RCLCPP_DEBUG(this->get_logger(), "conveyor: %s", info_->conveyor ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "squeeze: %s", info_->squeeze ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "squeeze_home: %s", info_->squeeze_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "roller_step: %s", info_->roller_step ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "roller_home: %s", info_->roller_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pill_gate_home: %s", info_->pill_gate_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_level_1: %s", info_->pkg_len_level_1 ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_level_2: %s", info_->pkg_len_level_2 ? "1" : "0");
    break;
  }
  }
}

void PackagingMachineNode::init_packaging_machine(void)
{
  status_->packaging_machine_state = PackagingMachineStatus::BUSY;

  RCLCPP_INFO(this->get_logger(), "init_packaging_machine start");
  ctrl_heater(HEATER_ON);
  rclcpp::sleep_for(DELAY_GENERAL_STEP);

  ctrl_stopper(STOPPER_PROTRUDE);
  wait_for_stopper(STOPPER_PROTRUDE_STATE);
  
  ctrl_material_box_gate(MTRL_BOX_GATE_OPEN);
  wait_for_material_box_gate(MTRL_BOX_GATE_OPEN_STATE);

  rclcpp::sleep_for(DELAY_MTRL_BOX_GATE);

  ctrl_material_box_gate(MTRL_BOX_GATE_CLOSE);
  wait_for_material_box_gate(MTRL_BOX_GATE_CLOSE_STATE);

  ctrl_stopper(STOPPER_SUNK);
  wait_for_stopper(STOPPER_SUNK_STATE);

  rclcpp::sleep_for(DELAY_GENERAL_STEP);

  for (uint8_t i = 0; i < CELLS_PER_DAY; i++)
  {
    ctrl_pill_gate(PILL_GATE_WIDTH, PILL_GATE_OPEN_DIR, MOTOR_ENABLE);
    wait_for_pill_gate();

    rclcpp::sleep_for(DELAY_GENERAL_STEP);
  }

  ctrl_pill_gate(PILL_GATE_WIDTH * NO_OF_PILL_GATES * PILL_GATE_CLOSE_MARGIN_FACTOR, PILL_GATE_CLOSE_DIR, MOTOR_ENABLE);
  wait_for_pill_gate();

  rclcpp::sleep_for(DELAY_GENERAL_STEP);

  init_printer();
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

    rclcpp::sleep_for(DELAY_PKG_DIS_WAIT_PRINTER);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN_FACTOR, PKG_DIS_FEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis();

    ctrl_pkg_dis(PKG_DIS_UNFEED_LEN, PKG_DIS_UNFEED_DIR, MOTOR_ENABLE);
    wait_for_pkg_dis();

    rclcpp::sleep_for(DELAY_PKG_DIS_BEFORE_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PUSH, MOTOR_ENABLE);
    wait_for_squeezer();

    rclcpp::sleep_for(DELAY_SQUEEZER);

    ctrl_squeezer(SQUEEZER_ACTION_PULL, MOTOR_ENABLE);
    wait_for_squeezer();
  }

  printer_.reset();
  RCLCPP_INFO(this->get_logger(), "printer destroyed");

  ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_DISABLE);
  rclcpp::sleep_for(DELAY_CONVEYOR_TESTING);
  ctrl_conveyor(CONVEYOR_SPEED, 0, CONVEYOR_FWD, MOTOR_ENABLE);
  
  for (uint8_t i = 0; i < DAYS; i++)
  {
    ctrl_roller(1, 0, MOTOR_ENABLE);
    wait_for_roller();
  }
  
  ctrl_roller(0, 1, MOTOR_ENABLE);
  wait_for_roller();

  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  RCLCPP_INFO(this->get_logger(), "init_packaging_machine end");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<PackagingMachineNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}