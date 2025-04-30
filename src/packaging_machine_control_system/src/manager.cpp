#include "packaging_machine_control_system/manager.hpp"

PackagingMachineManager::PackagingMachineManager(
  std::weak_ptr<rclcpp::Executor> executor,
  const std::string &node_name, 
  const std::string &node_namespace,
  const rclcpp::NodeOptions &options) 
: Node(std::move(node_name), node_namespace, options),
  executor_(executor)
{
  this->declare_parameter<int32_t>("no_of_pkg_mac", 0);
  this->get_parameter("no_of_pkg_mac", no_of_pkg_mac);

  status_sub_ = this->create_subscription<PackagingMachineStatus>(
    "packaging_machine_status", 
    10, 
    std::bind(&PackagingMachineManager::status_cb, this, _1));

  motor_status_sub_ = this->create_subscription<MotorStatus>(
    "motor_status", 
    10, 
    std::bind(&PackagingMachineManager::motor_status_cb, this, _1)); 

  info_sub_ = this->create_subscription<PackagingMachineInfo>(
    "info", 
    10, 
    std::bind(&PackagingMachineManager::info_cb, this, _1)); 

  packaging_result_sub_ = this->create_subscription<PackagingResult>(
    "packaging_result", 
    10, 
    std::bind(&PackagingMachineManager::packaging_result_cb, this, _1));

  action_client_manager_ = std::make_shared<rclcpp_components::ComponentManager>(
    executor_, 
    action_client_manager_node_name,
    rclcpp::NodeOptions().use_global_arguments(false));

  executor_->add_node(action_client_manager_);

  timer_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cli_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_ = this->create_service<PackagingOrderSrv>(
    packaging_order_service_name, 
    std::bind(&PackagingMachineManager::packaging_order_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  release_blk_srv_ = this->create_service<Trigger>(
    release_blocking_service_name, 
    std::bind(&PackagingMachineManager::release_blocking_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  income_box_srv_ = this->create_service<UInt8Srv>(
    income_box_service_name, 
    std::bind(&PackagingMachineManager::income_mtrl_box_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  con_box_srv_ = this->create_service<UInt8Srv>(
    con_box_service_name, 
    std::bind(&PackagingMachineManager::con_mtrl_box_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  manually_release_srv_ = this->create_service<Trigger>(
    manually_release_service_name, 
    std::bind(&PackagingMachineManager::manually_release_handle, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);  

  unbind_order_id_pub_ = this->create_publisher<UnbindRequest>("unbind_order_id", 10); 

  load_node_client_ = this->create_client<LoadNode>(
    load_node_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  unload_node_client_ = this->create_client<UnloadNode>(
    unload_node_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  list_node_client_ = this->create_client<ListNodes>(
    list_nodes_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  for (size_t i = 0; i < no_of_pkg_mac; i++)
  {
    const uint8_t id = i + 1;
    const std::string con_op_str = "/packaging_machine_" + std::to_string(id) + "/conveyor_operation";
    const std::string stop_op_str = "/packaging_machine_" + std::to_string(id) + "/stopper_operation";
    conveyor_stopper_client_[id] = std::make_pair(
      this->create_client<SetBool>(con_op_str, rmw_qos_profile_services_default, srv_cli_cbg_),
      this->create_client<SetBool>(stop_op_str, rmw_qos_profile_services_default, srv_cli_cbg_)
    );
    RCLCPP_DEBUG(this->get_logger(), "Conveyor Service %s Client is created", con_op_str.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Stopper Service %s Client is created", stop_op_str.c_str());
  }

  while (rclcpp::ok() && !load_node_client_->wait_for_service(1s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Load Node Service not available!");
  }
  while (rclcpp::ok() && !unload_node_client_->wait_for_service(1s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Unload Node Service not available!");
  }
  while (rclcpp::ok() && !list_node_client_->wait_for_service(1s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "List Nodes Service not available!");
  }

  for (const auto &cli_pair : conveyor_stopper_client_)
  {
    while (rclcpp::ok() && !cli_pair.second.first->wait_for_service(1s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "Machine [%d] Conveyor Service not available!", cli_pair.first);
    }
    while (rclcpp::ok() && !cli_pair.second.second->wait_for_service(1s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "Machine [%d] Stopper Service not available!", cli_pair.first);
    }
  }

  last_pkg_mac_scan_1 = 0;
  last_pkg_mac_scan_2 = 0;

  release_blocking_timer_ = this->create_wall_timer(
    1s, 
    std::bind(&PackagingMachineManager::release_blocking_cb, this),
    timer_cbg_);

  queue_handler_timer_ = this->create_wall_timer(
    1s, 
    std::bind(&PackagingMachineManager::queue_handler_cb, this),
    timer_cbg_);
  
  // second_machine_blocking_timer_ = this->create_wall_timer(
  //   1s, 
  //   std::bind(&PackagingMachineManager::second_machine_blocking_handler_cb, this),
  //   timer_cbg_);

  RCLCPP_INFO(this->get_logger(), "Packaging Machine Manager is up.");
  RCLCPP_INFO(this->get_logger(), "Total: %ld Packaging Machines are monitored", no_of_pkg_mac);
}

void PackagingMachineManager::release_blocking_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  if (release_blk_.empty())
    return;

  auto iter = std::find_if(pkg_mac_status_.rbegin(), pkg_mac_status_.rend(),
    [](const auto &status) {
      return status.second.conveyor_state == PackagingMachineStatus::UNAVAILABLE &&
             status.second.waiting_material_box == false;
  });

  if (iter == pkg_mac_status_.rend())
  {
    RCLCPP_DEBUG(this->get_logger(), "The conveyor of packaging machines are available and not waiting the material box, wait for next callback");
    // This operation is used for storing material box to container
    // if (income_box_.empty())
    // {
    //   release_blk_.pop(); 
    //   RCLCPP_INFO(this->get_logger(), "A element is poped form release_blk_");
    // }
    return;
  }

  const uint8_t target_id = iter->first;
  const auto &cli_pair = conveyor_stopper_client_[target_id];

  using ServiceSharedFutureAndRequestId = rclcpp::Client<SetBool>::SharedFutureAndRequestId;
  std::vector<ServiceSharedFutureAndRequestId> futures;

  using ServiceResponseFuture = rclcpp::Client<SetBool>::SharedFuture;
  auto response_received_cb = [this](ServiceResponseFuture future) {
    auto response = future.get();
    if (response) 
      RCLCPP_DEBUG(this->get_logger(), "Sent a operation request.");
    else 
    {
      const std::string err_msg = "Service call failed or returned no result";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    }
  };

  std::shared_ptr<SetBool::Request> conveyor_request = std::make_shared<SetBool::Request>();
  conveyor_request->data = true;
  auto conveyor_future = cli_pair.first->async_send_request(conveyor_request, response_received_cb);
  futures.push_back(std::move(conveyor_future));

  std::shared_ptr<SetBool::Request> stopper_request = std::make_shared<SetBool::Request>();
  stopper_request->data = false;
  auto stopper_future = cli_pair.second->async_send_request(stopper_request, response_received_cb);
  futures.push_back(std::move(stopper_future));

  RCLCPP_INFO(this->get_logger(), "Packaging Machine [%d] Conveyor and Stopper Service are called", target_id);

  bool success = true;
  for (const auto &future : futures)
  {
    std::future_status status = future.wait_for(1s);
    switch (status)
    {
    case std::future_status::ready:
      success &= true;
      break; 
    case std::future_status::timeout: {
      success &= false;
      const std::string err_msg = "The Operation Service is timeout.";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
      break; 
    }
    case std::future_status::deferred: {
      success &= false;
      const std::string err_msg = "The Operation Service is deferred.";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
      break;
    }
    }
  }

  if (success)
  {
    RCLCPP_INFO(this->get_logger(), "The conveyor of Packaging Machine [%d] is released successfully", target_id);
    release_blk_.pop();
    RCLCPP_INFO(this->get_logger(), "A element is poped form release_blk_");
  }
  else
    RCLCPP_ERROR(this->get_logger(), "The conveyor of Packaging Machine [%d] is released unsuccessfully", target_id);
}

void PackagingMachineManager::queue_handler_cb(void)
{
  const double TIME_GAP_THRESHOLD = 0.5;
  const std::lock_guard<std::mutex> lock(mutex_);

  rclcpp::Time curr_time = this->get_clock()->now();

  // case 1: Both queue are empty
  if (income_box_.empty() && packaging_order_.empty())
    return;

  // case 2: A packaging order is stored only
  if (income_box_.empty() && !packaging_order_.empty())
  {
    rclcpp::Duration time_diff = curr_time - packaging_order_.front().second;
    const double time_gap = time_diff.seconds();

    if (time_gap < TIME_GAP_THRESHOLD)
    {    
      RCLCPP_INFO(this->get_logger(), "The income material box signal maybe delayed. Try to handle in next callback."); 
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Large time gap detected: %.2f seconds", time_gap);
      packaging_order_.pop();
    }
    return;
  }

  // case 3: A income box signal is stored only
  if (!income_box_.empty() && packaging_order_.empty())
  {
    rclcpp::Duration time_diff = curr_time - income_box_.front().second;
    const double time_gap = time_diff.seconds();

    if (time_gap < TIME_GAP_THRESHOLD)
    {  
      RCLCPP_INFO(this->get_logger(), "The packaging order maybe delayed. Try to handle in next callback."); 
      return;
    }
  }

  // if (income_box_.front().first == packaging_order_.front().first)
  // {
  //   RCLCPP_INFO(this->get_logger(), "The income material box is handled by packaging order service");
  //   income_box_.pop();
  //   RCLCPP_INFO(this->get_logger(), "A element is poped form income_box_");
  //   packaging_order_.pop();
  //   RCLCPP_INFO(this->get_logger(), "A element is poped form packaging_order_");
  //   return;
  // }
  
  RCLCPP_INFO(this->get_logger(), "The incoming material box [%d] does not handled", income_box_.front().first); 
  RCLCPP_INFO(this->get_logger(), "The incoming material box should be passing the packaging machines");

  auto iter = std::find_if(pkg_mac_status_.rbegin(), pkg_mac_status_.rend(),
    [this](const auto& status) {
      return status.second.conveyor_state == PackagingMachineStatus::AVAILABLE &&
             status.second.waiting_material_box == false &&
             info_[status.first].conveyor == true;
  });

  if (iter == pkg_mac_status_.rend())
  {
    RCLCPP_ERROR(this->get_logger(), "Packaging Machines are unavailable!");
    return;
  }

  const uint8_t target_id = iter->first;
  const auto &cli_pair = conveyor_stopper_client_[target_id];

  using ServiceResponseFuture = rclcpp::Client<SetBool>::SharedFuture;
  auto response_received_cb = [this](ServiceResponseFuture future) {
    auto response = future.get();
    if (response) 
      RCLCPP_DEBUG(this->get_logger(), "Sent a operation request.");
    else 
    {
      const std::string err_msg = "Service call failed or returned no result";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    }
  };

  std::shared_ptr<SetBool::Request> request = std::make_shared<SetBool::Request>();
  request->data = true;
  auto future = cli_pair.second->async_send_request(request, response_received_cb);

  RCLCPP_INFO(this->get_logger(), "Packaging Machine [%d] Stopper Service are called", target_id);

  bool success = true;

  std::future_status status = future.wait_for(1s);
  switch (status)
  {
  case std::future_status::ready:
    break; 
  case std::future_status::timeout: {
    success = false;
    const std::string err_msg = "The Operation Service is timeout.";
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    break; 
  }
  case std::future_status::deferred: {
    success = false;
    const std::string err_msg = "The Operation Service is deferred.";
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    break;
  }
  }

  if (success)
  {
    RCLCPP_INFO(this->get_logger(), "The conveyor of Packaging Machine [%d] is blocked successfully", target_id);
    if (!income_box_.empty())
    {
      income_box_.pop();
      RCLCPP_INFO(this->get_logger(), "A element is poped form income_box_");
    }
  }
  else
    RCLCPP_ERROR(this->get_logger(), "The conveyor of Packaging Machine [%d] is blocked unsuccessfully", target_id);
}

// void PackagingMachineManager::second_machine_blocking_handler_cb(void)
// {
//   if (!income_box_.empty() || !packaging_order_.empty())
//     return;

//   const uint8_t target_id = 2;
//   if (pkg_mac_status_[target_id].waiting_material_box)
//     return;
  
//   const auto &cli_pair = conveyor_stopper_client_[target_id];

//   std::shared_ptr<SetBool::Request> request = std::make_shared<SetBool::Request>();
//   request->data = true;
//   auto future = cli_pair.second->async_send_request(request, response_received_cb);

//   RCLCPP_INFO(this->get_logger(), "Packaging Machine [%d] Stopper Service are called", target_id);

//   bool success = true;

//   std::future_status status = future.wait_for(500ms);
//   switch (status)
//   {
//   case std::future_status::ready:
//     break; 
//   case std::future_status::timeout: {
//     success = false;
//     const std::string err_msg = "The Operation Service is timeout.";
//     RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
//     break; 
//   }
//   case std::future_status::deferred: {
//     success = false;
//     const std::string err_msg = "The Operation Service is deferred.";
//     RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
//     break;
//   }
//   }

//   if (success)
//   {
//     RCLCPP_INFO(this->get_logger(), "The conveyor of Packaging Machine [%d] is blocked successfully", target_id);
//     if (!income_box_.empty())
//     {
//       income_box_.pop();
//       RCLCPP_INFO(this->get_logger(), "A element is poped form income_box_");
//     }
//   }
//   else
//     RCLCPP_ERROR(this->get_logger(), "The conveyor of Packaging Machine [%d] is blocked unsuccessfully", target_id);
  
// }

void PackagingMachineManager::status_cb(const PackagingMachineStatus::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  pkg_mac_status_[msg->packaging_machine_id] = *msg;
}

void PackagingMachineManager::motor_status_cb(const MotorStatus::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  motor_status_[msg->id] = *msg;
}

void PackagingMachineManager::info_cb(const PackagingMachineInfo::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  info_[msg->id] = *msg;
}
void PackagingMachineManager::packaging_result_cb(const PackagingResult::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  
  if (!msg->success)
  {
    RCLCPP_ERROR(this->get_logger(), "A packaging order return error.");
    // TODO: how to handle
    return;
  }

  // Find target in current clients
  auto target = std::find_if(curr_client_.begin(), curr_client_.end(),
    [msg](const std::pair<uint32_t, uint64_t>& entry) {
      return entry.first == msg->order_id;
    });

  // Publish unbind message
  UnbindRequest unbind_msg;
  unbind_msg.packaging_machine_id = msg->packaging_machine_id;
  unbind_msg.order_id = msg->order_id;
  unbind_msg.material_box_id = msg->material_box_id;
  unbind_order_id_pub_->publish(unbind_msg);

  if (target == curr_client_.end()) 
  {
    RCLCPP_ERROR(this->get_logger(), "The target action client is not found in manager.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "The target action client is found in manager");

  // Create and send list nodes request
  auto list_nodes_srv_request = std::make_shared<ListNodes::Request>();
  auto list_nodes_future = list_node_client_->async_send_request(list_nodes_srv_request);
  
  // Wait for and process list nodes response
  auto list_nodes_result = list_nodes_future.get();
  if (!list_nodes_result) 
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
    return;
  }

  // Find target unique ID in list nodes result
  auto target_unique_id = std::find(list_nodes_result->unique_ids.begin(), 
                                    list_nodes_result->unique_ids.end(), 
                                    target->second);

  if (target_unique_id == list_nodes_result->unique_ids.end()) 
  {
    RCLCPP_ERROR(this->get_logger(), "The action client is not found in ListNodes Service");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "The target unique_id is found in current loaded. Try to unload it now.");

  // Create and send unload node request
  auto unload_node_srv_request = std::make_shared<UnloadNode::Request>();
  unload_node_srv_request->unique_id = target->second;
  auto unload_node_future = unload_node_client_->async_send_request(unload_node_srv_request);

  // Wait for and process unload node response
  auto unload_node_result = unload_node_future.get();
  if (unload_node_result) 
  {
    curr_client_.erase(target);
    RCLCPP_INFO(this->get_logger(), "The action client is unloaded (unique_id: %ld)", target->second);
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
  }
}

void PackagingMachineManager::packaging_order_handle(
  const std::shared_ptr<PackagingOrderSrv::Request> request, 
  std::shared_ptr<PackagingOrderSrv::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "packaging order service handle");
  const std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(pkg_mac_status_.rbegin(), pkg_mac_status_.rend(),
    [](const auto& entry) {
        return entry.second.packaging_machine_state == PackagingMachineStatus::IDLE;
  });

  if (it == pkg_mac_status_.rend()) 
  {
    response->success = false;
    response->message = "Packaging Machines are not idle";
    RCLCPP_ERROR(this->get_logger(), "Packaging Machines are not idle");
    return;
  }

  const uint8_t target_machine_id = it->second.packaging_machine_id;
  const uint8_t mtrl_box_id = request->material_box_id;

  constexpr double TIME_GAP_THRESHOLD = 3.0;
  rclcpp::Time curr_time = this->get_clock()->now();

  bool is_empty = packaging_order_.empty();
  bool is_larger = !is_empty && (curr_time - packaging_order_.front().second).seconds() < TIME_GAP_THRESHOLD;

  if (is_empty || is_larger)
  {
    packaging_order_.push(std::pair(mtrl_box_id, curr_time));
    RCLCPP_INFO(this->get_logger(), "Pushed ID [%d] to packaging_order_", mtrl_box_id); 
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "The time gap of packaging request is closed"); 
    return;
  }

  auto load_node_srv_request = std::make_shared<LoadNode::Request>();
  load_node_srv_request->package_name = "packaging_machine_control_system";
  load_node_srv_request->plugin_name = "action_client::PackagingMachineActionClient";
  load_node_srv_request->node_name = "action_client_" + std::to_string(request->order_id);

  rcl_interfaces::msg::ParameterValue p1_v;
  p1_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p1_v.integer_value = target_machine_id;
  rcl_interfaces::msg::Parameter p1;
  p1.name = "packaging_machine_id";
  p1.value = p1_v;

  rcl_interfaces::msg::ParameterValue p2_v;
  p2_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p2_v.integer_value = request->order_id;
  rcl_interfaces::msg::Parameter p2;
  p2.name = "order_id";
  p2.value = p2_v;

  rcl_interfaces::msg::ParameterValue p3_v;
  p3_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p3_v.integer_value = request->material_box_id;
  rcl_interfaces::msg::Parameter p3;
  p3.name = "material_box_id";
  p3.value = p3_v;

  rcl_interfaces::msg::ParameterValue p_cn_name_v;
  p_cn_name_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_cn_name_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_cn_name_v.string_array_value[i] = request->print_info[i].cn_name;
  }
  rcl_interfaces::msg::Parameter p_cn_name;
  p_cn_name.name = "cn_name";
  p_cn_name.value = p_cn_name_v;

  rcl_interfaces::msg::ParameterValue p_en_name_v;
  p_en_name_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_en_name_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_en_name_v.string_array_value[i] = request->print_info[i].en_name;
  }
  rcl_interfaces::msg::Parameter p_en_name;
  p_en_name.name = "en_name";
  p_en_name.value = p_en_name_v;

  rcl_interfaces::msg::ParameterValue p_date_v;
  p_date_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_date_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_date_v.string_array_value[i] = request->print_info[i].date;
  }
  rcl_interfaces::msg::Parameter p_date;
  p_date.name = "date";
  p_date.value = p_date_v;

  rcl_interfaces::msg::ParameterValue p_time_v;
  p_time_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_time_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_time_v.string_array_value[i] = request->print_info[i].time;
  }
  rcl_interfaces::msg::Parameter p_time;
  p_time.name = "time";
  p_time.value = p_time_v;

  rcl_interfaces::msg::ParameterValue p_qr_code_v;
  p_qr_code_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_qr_code_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_qr_code_v.string_array_value[i] = request->print_info[i].qr_code;
  }
  rcl_interfaces::msg::Parameter p_qr_code;
  p_qr_code.name = "qr_code";
  p_qr_code.value = p_qr_code_v;

  rcl_interfaces::msg::ParameterValue p_drugs_v;
  p_drugs_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_drugs_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    auto join = [&](const std::vector<std::string>& vec, char delimiter) {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i) 
      {
        oss << vec[i];
        if (i != vec.size() - 1) 
          oss << delimiter; // Add # delimiter except for the last element
      }
      return oss.str();
    };
    p_drugs_v.string_array_value[i] = join(request->print_info[i].drugs, '#');
  }
  rcl_interfaces::msg::Parameter p_drugs;
  p_drugs.name = "drugs";
  p_drugs.value = p_drugs_v;

  load_node_srv_request->parameters.push_back(p1);
  load_node_srv_request->parameters.push_back(p2);
  load_node_srv_request->parameters.push_back(p3);
  load_node_srv_request->parameters.push_back(p_cn_name);
  load_node_srv_request->parameters.push_back(p_en_name);
  load_node_srv_request->parameters.push_back(p_date);
  load_node_srv_request->parameters.push_back(p_time);
  load_node_srv_request->parameters.push_back(p_qr_code);
  load_node_srv_request->parameters.push_back(p_drugs);

  using ServiceResponseFuture = rclcpp::Client<LoadNode>::SharedFuture;

  auto response_received_cb = [this, request, response](ServiceResponseFuture future) {
    auto srv_result = future.get();
    if (srv_result) 
    {
      std::pair<uint32_t, uint64_t> _pair(request->order_id, srv_result->unique_id);
      const std::lock_guard<std::mutex> lock(mutex_);
      curr_client_.push_back(_pair);
      RCLCPP_INFO(this->get_logger(), "Loaded a action client. unique_id: %ld ", srv_result->unique_id);
    } 
    else 
    {
      std::string err_msg = "Service call failed or returned no result";
      response->message = err_msg;
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    }
  };

  auto future = load_node_client_->async_send_request(load_node_srv_request, response_received_cb);

  std::future_status status = future.wait_for(1s);
  switch (status)
  {
  case std::future_status::ready:
    response->success = true;
    break; 
  default: {
    response->success = false;
    std::string err_msg = "The Loadnode Service is wait too long.";
    response->message = err_msg;
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    break;
  }
  }
}

void PackagingMachineManager::release_blocking_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;

  const std::lock_guard<std::mutex> lock(mutex_);

  constexpr double TIME_GAP_THRESHOLD = 3.0;
  rclcpp::Time curr_time = this->get_clock()->now();

  bool is_empty = release_blk_.empty();
  bool is_larger = !is_empty && (curr_time - release_blk_.front()).seconds() >= TIME_GAP_THRESHOLD;

  if (is_empty || is_larger)
  {
    release_blk_.push(curr_time);
    RCLCPP_INFO(this->get_logger(), "Pushed to release_blk_signal");
    response->success = true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "The release signal time gap of release signal is closed"); 
    response->success = false;
  }
}

void PackagingMachineManager::income_mtrl_box_handle(
  const std::shared_ptr<UInt8Srv::Request> request, 
  std::shared_ptr<UInt8Srv::Response> response)
{
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Handle a incoming material box service"); 

  const std::lock_guard<std::mutex> lock(mutex_);
  last_pkg_mac_scan_1 = request->data;

  bool is_empty = income_box_.empty();
  bool is_different = !is_empty && (income_box_.back().first != request->data);

  if (is_empty || is_different)
  {
    income_box_.push(std::make_pair(request->data, this->get_clock()->now()));
    RCLCPP_INFO(this->get_logger(), "Pushed ID [%d] to income_box_", request->data);
    response->success = true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "The incoming material box are repeated");
    response->success = false;
  }
}

void PackagingMachineManager::con_mtrl_box_handle(
  const std::shared_ptr<UInt8Srv::Request> request, 
  std::shared_ptr<UInt8Srv::Response> response)
{
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Handle a container material box service"); 

  const std::lock_guard<std::mutex> lock(mutex_);
  last_pkg_mac_scan_2 = request->data;
}

void PackagingMachineManager::manually_release_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Handle a manually release material box service"); 

  const std::lock_guard<std::mutex> lock(mutex_);

  if (!release_blk_.empty())
    release_blk_.pop();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<PackagingMachineManager>(
    exec, 
    "packaging_machine_manager",
    "",
    options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();
  rclcpp::shutdown();
}
