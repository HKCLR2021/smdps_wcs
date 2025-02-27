#include "wcs/dis_station_node.hpp"

DispenserStationNode::DispenserStationNode(const rclcpp::NodeOptions& options)
: Node("dis_station_node", options),
  status_(std::make_shared<DispenserStationStatus>()),
  cli_started_(std::atomic<bool>{false})
{
  this->declare_parameter<uint8_t>("id", 0);
  this->declare_parameter<std::string>("ip_start", "");
  this->declare_parameter<std::string>("port", "");
  this->declare_parameter<bool>("simulation", false);

  std::string ip_start;

  this->get_parameter("id", status_->id);
  this->get_parameter("ip_start", ip_start);
  this->get_parameter("port", port_);
  this->get_parameter("simulation", sim_);

  size_t last_dot_pos = ip_start.rfind(".");
  if (last_dot_pos != std::string::npos)
  {
    std::string last_octet_str = ip_start.substr(last_dot_pos + 1);
    int last_octet = std::stoi(last_octet_str);
    last_octet += status_->id;
    ip_ = ip_start.substr(0, last_dot_pos + 1) + std::to_string(last_octet);
  }
 
  for (size_t i = 0; i < status_->unit_status.size(); i++)
  {
    const uint8_t id = i + 1;
    status_->unit_status[i].id = id;
    status_->unit_status[i].enable = true;
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
    rclcpp::shutdown();
  }

  srv_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  status_pub_ = this->create_publisher<DispenserStationStatus>("/dispenser_station_status", 10);

  cli_thread_ = std::thread(std::bind(&DispenserStationNode::start_opcua_cli, this)); 
  wait_for_opcua_connection(200ms);

  status_timer_ = this->create_wall_timer(1s, std::bind(&DispenserStationNode::dis_station_status_cb, this));
  opcua_heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&DispenserStationNode::heartbeat_valid_cb, this));

  dis_req_srv_ = this->create_service<DispenseDrug>(
    "dispense_request", 
    std::bind(&DispenserStationNode::dis_req_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);
  
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

  initiate();
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
  const std::chrono::milliseconds freq = 100ms;
  const std::chrono::milliseconds TIMEOUT = 50ms;
  rclcpp::Rate loop_rate(freq); 

  wait_for_opcua_connection(freq);
  
  int16_t target_amt = 0;
  // Write Unit_Amount
  std::vector<std::future<opcua::StatusCode>> futures;
  for (size_t i = 0; i < req->content.size(); i++)
  {
    if (req->content[i].unit_id == 0 || req->content[i].unit_id > NO_OF_UNITS)
    {
      RCLCPP_ERROR(this->get_logger(), "incorrect unit_id in service request");
      continue;
    }

    if (req->content[i].amount <= 0)
      continue;

    opcua::Variant amt_var;
    amt_var = std::max(static_cast<int16_t>(req->content[i].amount), static_cast<int16_t>(0));

    target_amt += req->content[i].amount;

    std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, unit_amt_id[req->content[i].unit_id], amt_var, opcua::useFuture);
    futures.push_back(std::move(future));
  }
  RCLCPP_INFO(this->get_logger(), "Sent the amount of requested");

  // Write CmdRequest
  opcua::Variant cmd_req;
  cmd_req = true;
  std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, cmd_req_id, cmd_req, opcua::useFuture);
  futures.push_back(std::move(future));
  RCLCPP_INFO(this->get_logger(), "Sent the Command Request");

  bool all_good = wait_for_futures(futures);
  if (!all_good)
    RCLCPP_ERROR(this->get_logger(), "Error during amount write in %s", __FUNCTION__);

  // Read CmdAmount
  int16_t result;
  uint8_t i = 0;
  const uint16_t MAX_RETRIES = 100;
  for (; i < MAX_RETRIES && rclcpp::ok(); i++)
  {
    std::future<opcua::Result<opcua::Variant>> future = opcua::services::readValueAsync(cli, cmd_amt_id, opcua::useFuture);
    
    if (future.wait_for(TIMEOUT) != std::future_status::ready) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_WARN(this->get_logger(), "Aborted read due to ROS shutdown in %s", __FUNCTION__);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Read timeout after %ld ms, retrying (%d/%d)", TIMEOUT.count(), i + 1, MAX_RETRIES);
      continue;
    }

    const opcua::Result<opcua::Variant> &result_variant = future.get();
    if (result_variant.code() != UA_STATUSCODE_GOOD)
    {
      RCLCPP_ERROR(this->get_logger(), "readValueAsync error occur in %s", __FUNCTION__);
      continue;
    }

    std::optional<int16_t> val = result_variant.value().scalar<int16_t>();
    if (!val)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid cmd_amt value type in %s", __FUNCTION__);
      continue;
    }

    result = val.value();
    if (result == target_amt)
    {
      RCLCPP_INFO(this->get_logger(), "cmd_amt_val: %d matches target: %d", result, target_amt);
      break;
    }

    loop_rate.sleep();
  }

  if (i >= MAX_RETRIES)
  {
    RCLCPP_ERROR(this->get_logger(), "cmd_amt_val: %d does not matches target: %d", result, target_amt);
    RCLCPP_ERROR(this->get_logger(), "i >= MAX_RETRIES");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Verified the amount of request and target amount");

  // Write CmdExecute
  opcua::Variant cmd_exe;
  cmd_exe = true;
  std::future<opcua::StatusCode> cmd_exe_future = opcua::services::writeValueAsync(cli, cmd_exe_id, cmd_exe, opcua::useFuture);

  cmd_exe_future.wait();
  const opcua::StatusCode &cmd_exe_code = cmd_exe_future.get();
  if (cmd_exe_code != UA_STATUSCODE_GOOD)
  {
    RCLCPP_ERROR(this->get_logger(), "writeValueAsync error occur in %s", __FUNCTION__);
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

  // Read AmountDispensed
  std::future<opcua::Result<opcua::Variant>> amt_dis_future = opcua::services::readValueAsync(cli, amt_dis_id, opcua::useFuture);
  amt_dis_future.wait();

  const opcua::Result<opcua::Variant> &amt_dis_result = amt_dis_future.get();
  if (amt_dis_result.code() != UA_STATUSCODE_GOOD)
  {
    RCLCPP_ERROR(this->get_logger(), "readValueAsync error occur in %s", __FUNCTION__);
    return;
  }

  std::optional<int16_t> amt_dis_val = amt_dis_result.value().scalar<int16_t>();
  if (!amt_dis_val)
  {
    RCLCPP_ERROR(this->get_logger(), "amt_dis_val error occur in %s", __FUNCTION__);
    return;
  }

  if (amt_dis_val.value() != target_amt)
  {
    RCLCPP_INFO(this->get_logger(), "amt_dis_val.value(): %d", amt_dis_val.value());
    RCLCPP_INFO(this->get_logger(), "target_amt: %d", target_amt);
    RCLCPP_INFO(this->get_logger(), "Dispensed Amount is equal to request amount");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Verifing the amount of request and target amount");

  res->success = true;
  std::thread(std::bind(&DispenserStationNode::clear_cmd_req, this)).detach();
  std::thread(std::bind(&DispenserStationNode::initiate, this)).detach(); 
  RCLCPP_INFO(this->get_logger(), "Dispenser operation completed, proceeding...");
}

void DispenserStationNode::unit_req_handle(
  const std::shared_ptr<UnitRequest::Request> req, 
  std::shared_ptr<UnitRequest::Response> res)
{
  // auto valid_action = [&](const std::shared_ptr<UnitRequest::Request> req, const DispenserUnitStatus &unit_status) {
  //   switch (req->type) {
  //   case UnitType::BIN:
  //     return (unit_status.bin_opened && req->data == false) || 
  //            (unit_status.bin_closed && req->data == true);
  //   case UnitType::BAFFLE:
  //     return (unit_status.baffle_opened && req->data == false) || 
  //            (unit_status.baffle_closed && req->data == true);
  //   default:
  //     return false;
  //   }
  // };

  // bool action_criteria_satisfied = false;
  // std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  // lock.lock();
  // const uint8_t id_index = req->unit_id - 1;
  // const auto &unit_status = status_->unit_status[id_index];
  // action_criteria_satisfied = valid_action(req, unit_status);
  // lock.unlock();

  // if (!action_criteria_satisfied)
  // {
  //   res->success = false;
  //   res->message = "The target is either opening, closing, opened, or closed";
  //   return;
  // }

  opcua::Variant req_var;
  req_var = true;

  std::future<opcua::StatusCode> future;
  switch (req->type)
  {
  case UnitType::BIN:
    if (req->data)
      future = opcua::services::writeValueAsync(cli, bin_open_req_id[req->unit_id], req_var, opcua::useFuture);
    else
      future = opcua::services::writeValueAsync(cli, bin_close_req_id[req->unit_id], req_var, opcua::useFuture);
    break;
  case UnitType::BAFFLE:
    if (req->data)
      future = opcua::services::writeValueAsync(cli, baffle_open_req_id[req->unit_id], req_var, opcua::useFuture);
    else
      future = opcua::services::writeValueAsync(cli, baffle_close_req_id[req->unit_id], req_var, opcua::useFuture);
    break;
  default: {
    const std::string msg = "Unknow Type: %d";
    res->success = false;
    res->message = msg;
    RCLCPP_ERROR(this->get_logger(), msg.c_str(), req->type);
    return;
  }
  }

  future.wait();
  const opcua::StatusCode &code = future.get();

  if (code == UA_STATUSCODE_GOOD)
    res->success = true;
  else
  {
    res->success = false;
    RCLCPP_ERROR(this->get_logger(), "writeValueAsync error occur in %s", __FUNCTION__);
  }
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
