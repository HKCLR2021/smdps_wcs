#include "dis_station/dis_station_node.hpp"

DispenserStationNode::DispenserStationNode(const rclcpp::NodeOptions& options)
: Node("dis_station_node", options),
  status_(std::make_shared<DispenserStationStatus>()),
  cli_started_(std::atomic<bool>{false})
{
  this->declare_parameter<uint8_t>("id", 0);
  this->declare_parameter<std::string>("ip", "");
  this->declare_parameter<std::string>("port", "");
  this->declare_parameter<bool>("enable", false);
  this->declare_parameter<bool>("simulation", false);
  this->declare_parameter<std::vector<bool>>("dispenser_unit_enable", std::vector<bool>{});

  this->get_parameter("id", status_->id);
  this->get_parameter("ip", ip_);
  this->get_parameter("port", port_);
  this->get_parameter("enable", status_->enable);
  this->get_parameter("simulation", sim_);

  std::vector<bool> unit_enable = this->get_parameter("dispenser_unit_enable").as_bool_array();

  for (size_t i = 0; i < status_->unit_status.size(); i++)
  {
    const uint8_t id = i + 1;
    status_->unit_status[i].id = id;
    status_->unit_status[i].enable = unit_enable[i];
  }
  
  status_pub_ = this->create_publisher<DispenserStationStatus>("/dispenser_station_status", 10);
  dis_result_pub_ = this->create_publisher<DispenseResult>("/dispense_result", 10);
  
  heartbeat_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  status_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  restart_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ree_srv_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  
  status_timer_ = this->create_wall_timer(1s, std::bind(&DispenserStationNode::dis_station_status_cb, this), status_cbg_);

  dis_req_srv_ = this->create_service<DispenseDrug>(
    "dispense_request", 
    std::bind(&DispenserStationNode::dis_req_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  if (!status_->enable)
  {
    // The node will still be running
    RCLCPP_INFO(this->get_logger(), "Dispenser Station [%d] is disable", status_->id);
    return;
  }

  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    const uint8_t id = i + 1;
    unit_amt_id[id]         = {ns_ind, rev_prefix + "Unit" + std::to_string(id) + "Amount"};

    bin_open_req_id[id]     = {ns_ind, rev_prefix + "Bin" + std::to_string(id) + "OpenRequest"};
    bin_close_req_id[id]    = {ns_ind, rev_prefix + "Bin" + std::to_string(id) + "CloseRequest"};
    baffle_open_req_id[id]  = {ns_ind, rev_prefix + "Baffle" + std::to_string(id) + "OpenRequest"};
    baffle_close_req_id[id] = {ns_ind, rev_prefix + "Baffle" + std::to_string(id) + "CloseRequest"};

    unit_lack_id[id]        = {ns_ind, send_prefix + "Unit" + std::to_string(id) + "Lack"};

    bin_opened_id[id]       = {ns_ind, send_prefix + "Bin" + std::to_string(id) + "Opened"};
    bin_closed_id[id]       = {ns_ind, send_prefix + "Bin" + std::to_string(id) + "Closed"};
    baffle_opened_id[id]    = {ns_ind, send_prefix + "Baffle" + std::to_string(id) + "Opened"};
    baffle_closed_id[id]    = {ns_ind, send_prefix + "Baffle" + std::to_string(id) + "Closed"};
  }

  if (!init_opcua_cli())
  {
    RCLCPP_ERROR(this->get_logger(), "Initiate the OPCUA Client error occurred");
    std::this_thread::sleep_for(1s);
    rclcpp::shutdown();
  }

  opcua_heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&DispenserStationNode::heartbeat_valid_cb, this), heartbeat_cbg_);

  bin_req_srv_ = this->create_service<UnitRequest>(
    "bin_operation_request", 
    std::bind(&DispenserStationNode::unit_req_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  baffle_req_srv_ = this->create_service<UnitRequest>(
    "baffle_operation_request", 
    std::bind(&DispenserStationNode::unit_req_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  init_bin_srv_ = this->create_service<Trigger>(
    "init_bin_request", 
    std::bind(&DispenserStationNode::init_bin_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  init_baffle_srv_ = this->create_service<Trigger>(
    "init_baffle_request", 
    std::bind(&DispenserStationNode::init_baffle_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  reset_srv_ = this->create_service<Trigger>(
    "reset_request", 
    std::bind(&DispenserStationNode::reset_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    ree_srv_ser_cbg_);

  init_srv_ = this->create_service<Trigger>(
    "init_request", 
    std::bind(&DispenserStationNode::init_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    ree_srv_ser_cbg_);

  restart_srv_ = this->create_service<Trigger>(
    "restart_request", 
    std::bind(&DispenserStationNode::restart_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    restart_cbg_);

  cli_thread_ = std::thread(std::bind(&DispenserStationNode::start_opcua_cli, this));
  
  try 
  {
    wait_for_opcua_connection(std::chrono::milliseconds(1000), std::chrono::seconds(60));
  } 
  catch (const std::runtime_error& e) 
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to OPC UA: %s", e.what());
    std::this_thread::sleep_for(1s);
    rclcpp::shutdown();
  }

  initiate();
  std::this_thread::sleep_for(500ms);
  reset();

  RCLCPP_INFO(this->get_logger(), "OPCUA Server: %s", form_opcua_url().c_str());
  RCLCPP_INFO(this->get_logger(), "Dispenser Station Node [%d] is up", status_->id);
}

DispenserStationNode::~DispenserStationNode()
{
  if (cli_started_.load()) 
  {
    cli_started_.store(false);
    cli.stop();
  }
  if (cli_thread_.joinable()) 
  {
    cli_thread_.join();
  }

  RCLCPP_INFO(this->get_logger(), "OPC UA Thread [%d] is deleted", status_->id);
}

void DispenserStationNode::dis_station_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_pub_->publish(*status_);
}

void DispenserStationNode::heartbeat_valid_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  heartbeat_counter_++;
  
  if (heartbeat_counter_ > MAX_HB_COUNT)
  {
    status_->heartbeat = false;
    RCLCPP_ERROR(this->get_logger(), "Lost connection to %s", ip_.c_str());
  }
}

void DispenserStationNode::dis_req_handle(
  const std::shared_ptr<DispenseDrug::Request> req, 
  std::shared_ptr<DispenseDrug::Response> res)
{
  if (!status_->enable)
    return;

  if (req->content.size() == 0)
  {
    res->success = true;
    RCLCPP_ERROR(this->get_logger(), "Test the dispense request successfully!");
    return;
  }

  int16_t target_amt = 0;
  // Write Unit_Amount
  // std::vector<std::future<opcua::StatusCode>> futures;
  std::string dispense_str;
  for (size_t i = 0; i < req->content.size(); i++)
  {
    if (req->content[i].unit_id == 0 || req->content[i].unit_id > NO_OF_UNITS)
    {
      RCLCPP_ERROR(this->get_logger(), "incorrect unit_id in service request");
      continue;
    }

    // if (!status_->unit_status[i].enable)
    //   continue;

    if (req->content[i].amount <= 0)
      continue;

    // opcua::Variant amt_var;
    const int16_t tmp = std::max(static_cast<int16_t>(req->content[i].amount), static_cast<int16_t>(0));
    // amt_var = tmp;
    target_amt += tmp;

    dispense_str += "[id: " + std::to_string(req->content[i].unit_id) + ", amount: " + std::to_string(req->content[i].amount) + "] ";
    // std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, unit_amt_id[req->content[i].unit_id], amt_var, opcua::useFuture);
    // futures.push_back(std::move(future));

    // Write UnitAmount
    if (!write_opcua_value(unit_amt_id[req->content[i].unit_id], tmp))
    {
      RCLCPP_ERROR(this->get_logger(), "Error occur in Write Unit%dAmount", tmp);
      return;
    }
  }
  dispense_str = "Dispense: " + dispense_str;
  RCLCPP_INFO(this->get_logger(), dispense_str.c_str());
  RCLCPP_INFO(this->get_logger(), "Sent the amount of requested");

  // bool all_good = wait_for_futures(futures);
  // if (!all_good)
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Error during amount write in %s", __FUNCTION__);
  //   return;
  // }

  // Write CmdRequest
  if (!cmd_req(true))
  {
    RCLCPP_ERROR(this->get_logger(), "Error during command request write in %s", __FUNCTION__);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Sent the Command Request");

  // Read CmdAmount
  uint16_t tries = 0;
  int16_t result = 0;
  const uint16_t MAX_RETRIES = 100;
  for (; tries < MAX_RETRIES && rclcpp::ok(); tries++)
  {    
    if (rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting the CmdAmount signal...");

      std::unique_lock<std::mutex> lock(cmd_amt_signal.cv_mutex_);
      cmd_amt_signal.cv_.wait(lock, [this]() { 
        return cmd_amt_signal.is_triggered_; 
      });

      result = cmd_amt_signal.val;
      cmd_amt_signal.is_triggered_ = false;
      cmd_amt_signal.val = 0;
    }

    RCLCPP_INFO(this->get_logger(), "The CmdAmount: %d", result);
    RCLCPP_INFO(this->get_logger(), "The target amount: %d", target_amt);

    if (result == target_amt)
    {
      RCLCPP_INFO(this->get_logger(), "cmd_amt_val: %d matches target: %d", result, target_amt);
      break;
    }
  }

  if (tries >= MAX_RETRIES)
  {
    RCLCPP_ERROR(this->get_logger(), "cmd_amt_val: %d does not matches target: %d", result, target_amt);
    RCLCPP_ERROR(this->get_logger(), "i >= MAX_RETRIES");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Verified the amount of request and target amount");

  // Write CmdExecute
  if (!write_opcua_value(cmd_exe_id, true))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in Write CmdExecute");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sent the Command Execute");

  // wait forever until the dispenser station is completed
  if (rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Waiting the completion signal...");
    std::unique_lock<std::mutex> lock(com_signal.cv_mutex_);
    com_signal.cv_.wait(lock, [this]() { 
      return com_signal.is_triggered_; 
    });
    com_signal.is_triggered_ = false;
  }

  // for waiting the AmountDispensed value
  std::this_thread::sleep_for(100ms);

  // Read AmountDispensed
  std::shared_ptr<int16_t> amt_dis_val = std::make_shared<int16_t>();
  if (!read_opcua_value(amt_dis_id, amt_dis_val))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in Read AmountDispensed");
    return;
  }
  
  if (*amt_dis_val == target_amt)
  {
    RCLCPP_INFO(this->get_logger(), "Dispensed Amount is equal to request amount");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "*amt_dis_val: %d", *amt_dis_val);
    RCLCPP_ERROR(this->get_logger(), "target_amt: %d", target_amt);
    RCLCPP_ERROR(this->get_logger(), "Dispensed Amount is not equal to request amount");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Verified the amount of request and target amount");

  // Reset CmdRequest
  if (!cmd_req(false))
  {
    RCLCPP_ERROR(this->get_logger(), "Error during command request write in %s", __FUNCTION__);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Reset the Command Request");

  std::thread(std::bind(&DispenserStationNode::initiate, this)).detach(); 
  RCLCPP_INFO(this->get_logger(), "Dispenser operation completed, proceeding...");
  
  std::this_thread::sleep_for(600ms); // Caution: be careful to set this delay
  
  res->success = true;

  DispenseResult msg;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    msg.id = status_->id;
  }
  msg.success = res->success;
  dis_result_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Dispense result published");
}

void DispenserStationNode::unit_req_handle(
  const std::shared_ptr<UnitRequest::Request> req, 
  std::shared_ptr<UnitRequest::Response> res)
{
  bool success;

  switch (req->type)
  {
  case UnitType::BIN:
    if (req->data)
      success = write_opcua_value(bin_open_req_id[req->unit_id], true);
    else
      success = write_opcua_value(bin_close_req_id[req->unit_id], true);
    break;
  case UnitType::BAFFLE:
    if (req->data)
      success = write_opcua_value(baffle_open_req_id[req->unit_id], true);
    else
      success = write_opcua_value(baffle_close_req_id[req->unit_id], true);
    break;
  default: {
    const std::string msg = "Unknow Type: %d";
    res->success = false;
    res->message = msg;
    RCLCPP_ERROR(this->get_logger(), msg.c_str(), req->type);
    return;
  }
  }

  res->success = success;
  if (!success)
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
  }
}

void DispenserStationNode::init_bin_handle(
  const std::shared_ptr<Trigger::Request> req, 
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;
  std::thread{std::bind(&DispenserStationNode::test_bin, this)}.detach();
  res->success = true;
}

void DispenserStationNode::init_baffle_handle(
  const std::shared_ptr<Trigger::Request> req, 
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;
  std::thread{std::bind(&DispenserStationNode::test_baffle, this)}.detach();
  res->success = true;
}

void DispenserStationNode::reset_handle(
  const std::shared_ptr<Trigger::Request> req, 
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;
  std::thread{std::bind(&DispenserStationNode::reset, this)}.detach();
  res->success = true;
}

void DispenserStationNode::init_handle(
  const std::shared_ptr<Trigger::Request> req, 
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;
  std::thread{std::bind(&DispenserStationNode::initiate, this)}.detach();
  res->success = true;
}

void DispenserStationNode::restart_handle(
  const std::shared_ptr<Trigger::Request> req, 
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;

  std::thread{[&] {
    RCLCPP_ERROR(this->get_logger(), "This node will be shurdown after 1 second");
    std::this_thread::sleep_for(1s);
    rclcpp::shutdown();
  }}.detach();

  res->success = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<DispenserStationNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}
