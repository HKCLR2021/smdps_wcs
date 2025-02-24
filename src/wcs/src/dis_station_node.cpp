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
    status_->unit_status[i].id = i + 1;
    status_->unit_status[i].enable = true;
  }

  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    unit_amt_id[i]         = {ns_ind, rev_prefix + "Unit" + std::to_string(i+1) + "Amount"};

    bin_open_req_id[i]     = {ns_ind, rev_prefix + "Bin" + std::to_string(i+1) + "OpenRequest"};
    bin_close_req_id[i]    = {ns_ind, rev_prefix + "Bin" + std::to_string(i+1) + "OpenRequest"};
    baffle_open_req_id[i]  = {ns_ind, rev_prefix + "Baffle" + std::to_string(i+1) + "OpenRequest"};
    baffle_close_req_id[i] = {ns_ind, rev_prefix + "Baffle" + std::to_string(i+1) + "OpenRequest"};

    unit_lack_id[i]        = {ns_ind, send_prefix + "Unit" + std::to_string(i+1) + "Lack"};

    bin_opening_id[i]      = {ns_ind, send_prefix + "Bin" + std::to_string(i+1) + "Opening"};
    bin_opened_id[i]       = {ns_ind, send_prefix + "Bin" + std::to_string(i+1) + "Opened"};
    bin_closing_id[i]      = {ns_ind, send_prefix + "Bin" + std::to_string(i+1) + "Closing"};
    bin_closed_id[i]       = {ns_ind, send_prefix + "Bin" + std::to_string(i+1) + "Closed"};
    baffle_opening_id[i]   = {ns_ind, send_prefix + "Baffle" + std::to_string(i+1) + "Opening"};
    baffle_opened_id[i]    = {ns_ind, send_prefix + "Baffle" + std::to_string(i+1) + "Opened"};
    baffle_closing_id[i]   = {ns_ind, send_prefix + "Baffle" + std::to_string(i+1) + "Closing"};
    baffle_closed_id[i]    = {ns_ind, send_prefix + "Baffle" + std::to_string(i+1) + "Closed"};
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
  RCLCPP_DEBUG(this->get_logger(), "%s is working", __FUNCTION__);
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
  std::chrono::milliseconds freq = 100ms;
  rclcpp::Rate loop_rate(freq); 

  wait_for_opcua_connection(freq);
  
  std::vector<std::future<opcua::StatusCode>> futures;
  for (size_t i = 0; i < req->content.size(); i++)
  {
    if (req->content[i].unit_id == 0 || req->content[i].unit_id > NO_OF_UNITS)
    {
      RCLCPP_ERROR(this->get_logger(), "incorrect unit_id in service request");
      continue;
    }

    opcua::Variant amt_var;
    amt_var = static_cast<int16_t>(req->content[i].amount);

    std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, unit_amt_id[req->content[i].unit_id - 1], amt_var, opcua::useFuture);
    futures.push_back(std::move(future));
  }

  for (auto &future: futures)
  {
    future.wait();
    const opcua::StatusCode &code = future.get();
    if (code != UA_STATUSCODE_GOOD)
      RCLCPP_ERROR(this->get_logger(), "writeValueAsync error occur in %s", __FUNCTION__);
  }

  opcua::Variant cmd_req;
  cmd_req = true;
  std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, cmd_req_id, cmd_req, opcua::useFuture);

  future.wait();
  const opcua::StatusCode &code = future.get();
  if (code != UA_STATUSCODE_GOOD)
    RCLCPP_ERROR(this->get_logger(), "writeValueAsync error occur in %s", __FUNCTION__);

  // wait forever until the dispenser station is completed
  while (rclcpp::ok())
  {
    std::future<opcua::Result<opcua::Variant>> future = opcua::services::readValueAsync(cli, completed_id, opcua::useFuture);
    future.wait();
    const opcua::Result<opcua::Variant> &result = future.get();

    if (result.code() == UA_STATUSCODE_GOOD)
    {
      std::optional<bool> val = result.value().scalar<bool>();
      if (val && *val)
      {
        RCLCPP_INFO(this->get_logger(), "Submit a dispense request successfully");
        res->success = true;
        break;
      }
    }
    else
      RCLCPP_ERROR(this->get_logger(), "Read result with status code: %s", std::to_string(result.code()).c_str());

    loop_rate.sleep();
  }

  RCLCPP_INFO(this->get_logger(), "%s is completed", __FUNCTION__);
}

void DispenserStationNode::unit_req_handle(
  const std::shared_ptr<UnitRequest::Request> req, 
  std::shared_ptr<UnitRequest::Response> res)
{
  auto valid_action = [&](const std::shared_ptr<UnitRequest::Request> req, const DispenserUnitStatus &unit_status) {
    switch (req->type) {
    case UnitType::BIN:
      return (unit_status.bin_opened && req->data == false) || 
             (unit_status.bin_closed && req->data == true);
    case UnitType::BAFFLE:
      return (unit_status.baffle_opened && req->data == false) || 
             (unit_status.baffle_closed && req->data == true);
    default:
      return false;
    }
  };

  bool action_criteria_satisfied = false;
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  lock.lock();
  const auto &unit_status = status_->unit_status[req->unit_id - 1];
  action_criteria_satisfied = valid_action(req, unit_status);
  lock.unlock();

  if (!action_criteria_satisfied)
  {
    res->success = false;
    res->message = "The target is either opening, closing, opened, or closed";
    return;
  }

  opcua::Variant req_var;
  req_var = req->data;

  std::future<opcua::StatusCode> future;
  switch (req->type)
  {
  case UnitType::BIN:
    if (req->data)
      future = opcua::services::writeValueAsync(cli, bin_open_req_id[req->unit_id - 1], req_var, opcua::useFuture);
    else
      future = opcua::services::writeValueAsync(cli, bin_close_req_id[req->unit_id - 1], req_var, opcua::useFuture);
    break;
  case UnitType::BAFFLE:
    if (req->data)
      future = opcua::services::writeValueAsync(cli, baffle_open_req_id[req->unit_id - 1], req_var, opcua::useFuture);
    else
      future = opcua::services::writeValueAsync(cli, baffle_close_req_id[req->unit_id - 1], req_var, opcua::useFuture);
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
  {
    res->success = true;
  }
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
