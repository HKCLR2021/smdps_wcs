#include "wcs/prod_line_ctrl.hpp"

ProdLineCtrl::ProdLineCtrl(const rclcpp::NodeOptions& options)
: Node("prod_line_ctrl", options), 
  count_(0),
  svr_started_(std::atomic<bool>{false})
{
  this->declare_parameter<std::string>("hkclr_ip", "");
  this->declare_parameter<int>("hkclr_port", 0);
  this->declare_parameter<std::string>("jinli_ip", "");
  this->declare_parameter<int>("jinli_port", 0);

  this->get_parameter("hkclr_ip", httpsvr_ip_);
  this->get_parameter("hkclr_port", httpsvr_port_);
  this->get_parameter("jinli_ip", jinli_ip_);
  this->get_parameter("jinli_port", jinli_port_);

  RCLCPP_INFO(this->get_logger(), "jinli HTTP server: %s:%d", jinli_ip_.c_str(), jinli_port_);
  RCLCPP_INFO(this->get_logger(), "hkclr HTTP server: %s:%d", httpsvr_ip_.c_str(), httpsvr_port_);

  rclcpp::QoS dispense_request_qos_profile(10);
  dispense_request_qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  dispense_request_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

  pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  dis_err_pub_ = this->create_publisher<DispensingError>("dispensing_error", 10);
  dis_req_pub_ = this->create_publisher<DispensingDetail>("dispense_request", dispense_request_qos_profile);
  cleaning_mac_scan_pub_ = this->create_publisher<CleaningMachineScanner>("cleaning_machine_scanner", 10);
  order_compl_pub_ = this->create_publisher<OrderCompletion>("order_completion", 10);
  mtrl_box_amt_pub_ = this->create_publisher<ContainerInfo>("container_info", 10);

  pkg_mac_status_sub_ = this->create_subscription<PackagingMachineStatus>(
    "packaging_machine_status", 
    10, 
    std::bind(&ProdLineCtrl::pkg_mac_status_cb, this, _1));

  timer_ = this->create_wall_timer(1s, std::bind(&ProdLineCtrl::timer_callback, this));
  mtrl_box_amt_timer_ = this->create_wall_timer(1s, std::bind(&ProdLineCtrl::mtrl_box_amt_container_cb, this));
  // mtrl_box_info_timer_ = this->create_wall_timer(1s, std::bind(&ProdLineCtrl::mtrl_box_info_cb, this));

  this->action_server_ = rclcpp_action::create_server<NewOrder>(
    this,
    "new_order",
    std::bind(&ProdLineCtrl::handle_goal, this, _1, _2),
    std::bind(&ProdLineCtrl::handle_cancel, this, _1),
    std::bind(&ProdLineCtrl::handle_accepted, this, _1));

  if (!init_httpsvr())
  {
    RCLCPP_INFO(this->get_logger(), "init_httpsvr error occurred");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "Production Line Control is up");
}

ProdLineCtrl::~ProdLineCtrl()
{
  if (svr_started_.load()) 
  {
    httpsvr_->wait_until_ready();
    httpsvr_->stop();
  }
  if (httpsvr_thread_.joinable()) 
  {
    httpsvr_thread_.join();
  }
}

void ProdLineCtrl::mtrl_box_amt_container_cb(void)
{
  nlohmann::json res_json;

  if (get_material_box_amt(res_json)) 
  {
    RCLCPP_DEBUG(this->get_logger(), "\n%s", res_json.dump().c_str());
    ContainerInfo msg;
    msg.header.stamp = this->get_clock()->now();
    msg.amount = static_cast<uint8_t>(res_json["amount"]);
    mtrl_box_amt_pub_->publish(msg);
  } 
}

void ProdLineCtrl::mtrl_box_info_cb(void)
{
  nlohmann::json res_json;
  
  if (get_material_box_info(res_json)) 
  {
    RCLCPP_DEBUG(this->get_logger(), "\n%s", res_json.dump().c_str());
    for (const auto &x : res_json["materialBoxs"])
    {
      nlohmann::json mtrl_box_res_json;
      const httplib::Params params = {
        {"materialBoxId", x["id"]},
        {"isCompleted", "1"}
      };

      if (get_cell_info_by_id(params, mtrl_box_res_json))
      {
        MaterialBoxStatus msg;
        msg.header.stamp = this->get_clock()->now();
        msg.id = x["id"];
        msg.location = x["location"];
        msg.status = MaterialBoxStatus::STATUS_ERROR;
        
        if (msg.material_box.slots.size() == mtrl_box_res_json["cells"].size())
        {
          // for (size_t i = 0; i < mtrl_box_res_json["cells"].size(); i++)
          // {
          //   MaterialBoxSlot slot;
          //   for (const auto &drug : mtrl_box_res_json["cells"]["drugs"])
          //   { 
          //     DispensingDetail dis_detail_msg;
          //     for (const auto &location : drug["locations"]) // The length must be 1
          //     {
          //       dis_detail_msg.location.dispenser_station = location["dispenserStation"];
          //       dis_detail_msg.location.dispenser_unit = location["dispenserUnit"];
          //     }
          //     dis_detail_msg.amount = drug["amount"];
          //     slot.dispensing_detail.push_back(dis_detail_msg);
          //   }
          //   msg.material_box.slots[i] = slot;
          //   RCLCPP_INFO(this->get_logger(), "a cell is added to msg, i: %ld", i);
          // }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "size not equal");
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "material box not found");
      }
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "%s error ocurred", __FUNCTION__);
  }
}

void ProdLineCtrl::pkg_mac_status_cb(const PackagingMachineStatus::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  pkg_mac_status[msg->packaging_machine_id] = *msg;
}

bool ProdLineCtrl::get_material_box_info(nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  auto res = cli.Get(mtrl_box_info_url_);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::get_material_box_info_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = cli.Get(mtrl_box_info_by_id_url_, params, headers);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::get_cell_info_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = cli.Get(cells_info_by_id_url_, params, headers);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::get_material_box_amt(nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  auto res = cli.Get(mtrl_box_amt_url_);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::new_order(const nlohmann::json &req_json, nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  httplib::Headers headers = {
    { "Content-Type", "application/json" }
  };
  std::string req_body = req_json.dump();

  auto res = cli.Post(
    new_order_url_, 
    headers,
    req_body, 
    "application/json");

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::get_order_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = cli.Get(order_by_id_url_, params, headers);

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::dispense_result(const nlohmann::json &req_json, nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  httplib::Headers headers = {
    { "Content-Type", "application/json" }
  };
  std::string req_body = req_json.dump();

  auto res = cli.Post(
    dis_result_url_, 
    headers,
    req_body, 
    "application/json");

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::health(nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 
  
  auto res = cli.Get(health_url_);

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

rclcpp_action::GoalResponse ProdLineCtrl::handle_goal(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const NewOrder::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received a goal, start to proceess");

  // FIXME: quick fix, assume no error
  if (0)
    return rclcpp_action::GoalResponse::REJECT;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ProdLineCtrl::handle_cancel(
  const std::shared_ptr<GaolHandlerNewOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ProdLineCtrl::handle_accepted(const std::shared_ptr<GaolHandlerNewOrder> goal_handle)
{
  std::thread{std::bind(&ProdLineCtrl::order_execute, this, _1), goal_handle}.detach();
}

void ProdLineCtrl::order_execute(const std::shared_ptr<GaolHandlerNewOrder> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NewOrder::Feedback>();
  auto &running = feedback->running;
  auto result = std::make_shared<NewOrder::Result>();
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  nlohmann::json req_json, res_json;
  for (size_t i = 0; i < goal->request.material_box.slots.size(); i++) 
  {
    auto &slots_i = goal->request.material_box.slots[i];
    nlohmann::json _cell;

    for (const auto &drugs_j : slots_i.drugs)
    {
      nlohmann::json _drug;
      _drug["drugId"] = drugs_j.drug_id;
      _drug["amount"] = drugs_j.amount;

      for (const auto &location_j : drugs_j.locations)
      {
        nlohmann::json location = {
          {"dispenserStation", location_j.dispenser_station},
          {"dispenserUnit", location_j.dispenser_unit}
        };
        _drug["locations"].push_back(location);
      }
      
      _cell["drugs"].push_back(_drug);
    }

    req_json["cells"].push_back(_cell);
    RCLCPP_INFO(this->get_logger(), "a cell is added, i: %ld", i);
  }

  running = true;

  if (new_order(req_json, res_json))
  {
    RCLCPP_INFO(this->get_logger(), "a new order is sent, waiting for material box id...");
    const httplib::Params params = {
      {"orderId", res_json["orderId"]}
    };

    uint16_t retries = 0;
    const uint16_t MAX_RETRY = 60; // FIXME
    rclcpp::Rate loop_rate(1s); 

    for (; retries < MAX_RETRY && rclcpp::ok(); ++retries) 
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for the material box id (%d times)...", retries + 1);
      res_json.clear();
      if (get_order_by_id(params, res_json))
      {
        if (static_cast<int>(res_json["sortBoder"]["borderMaterialBoxId"]) != 0)
        {
          result->response.material_box_id = static_cast<int>(res_json["sortBoder"]["borderMaterialBoxId"]);
          result->response.result = true;
          break;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Received a material box id is 0");
        }
      }
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    if (retries >= MAX_RETRY)
    {
      RCLCPP_ERROR(this->get_logger(), "retries (%d) >= MAX_RETRY", retries);
    }
  }

  if (rclcpp::ok()) 
  {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

inline const std::string ProdLineCtrl::from_url(const std::string resource)
{
  return api_ + version_ + resource;
}

inline bool ProdLineCtrl::verify_params(const httplib::Request &req, const std::vector<std::string> &keys) {
  return std::all_of(keys.begin(), keys.end(),
    [&req](auto p) {
      return req.has_param(p);
    }
  );
}

std::string ProdLineCtrl::dump_headers(const httplib::Headers &headers) 
{
  std::string s;
  char buf[BUFSIZ];

  for (const auto &x : headers) 
  {
    snprintf(buf, sizeof(buf), "%s: %s\n", x.first.c_str(), x.second.c_str());
    s += buf;
  }

  return s;
}

std::string ProdLineCtrl::dump_multipart_files(const httplib::MultipartFormDataMap &files) 
{
  std::string s;
  char buf[BUFSIZ];
  s += "--------------------------------\n";
  for (const auto &x : files) 
  {
    const auto &name = x.first;
    const auto &file = x.second;
    snprintf(buf, sizeof(buf), "name: %s\n", name.c_str());
    s += buf;
    snprintf(buf, sizeof(buf), "filename: %s\n", file.filename.c_str());
    s += buf;
    snprintf(buf, sizeof(buf), "content type: %s\n", file.content_type.c_str());
    s += buf;
    snprintf(buf, sizeof(buf), "text length: %zu\n", file.content.size());
    s += buf;
    s += "----------------\n";
  }

  return s;
}

bool ProdLineCtrl::init_httpsvr(void)
{
  try
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    httpsvr_ = std::make_shared<httplib::Server>();

    httpsvr_->set_logger(std::bind(&ProdLineCtrl::logger_handler, this, _1, _2));
    httpsvr_->set_error_handler(std::bind(&ProdLineCtrl::error_handler, this, _1, _2));
    httpsvr_->set_exception_handler(std::bind(&ProdLineCtrl::exception_handler, this, _1, _2, _3));
    httpsvr_->set_pre_routing_handler(std::bind(&ProdLineCtrl::pre_routing_handler, this, _1, _2));
    httpsvr_->set_payload_max_length(1024 * 1024 * 4); // 4MB

    httpsvr_->Get(from_url(health_), std::bind(&ProdLineCtrl::health_handler, this, _1, _2));
    httpsvr_->Post(from_url(abnormal_dispensation_), std::bind(&ProdLineCtrl::abnormal_dispensation_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(abnormal_device_), std::bind(&ProdLineCtrl::abnormal_device_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(dispense_request_), std::bind(&ProdLineCtrl::dispense_request_handler, this, _1, _2, _3));
    // httpsvr_->Post(from_url(packaging_request_), std::bind(&ProdLineCtrl::packaging_request_handler, this, _1, _2));
    httpsvr_->Get(from_url(packaging_info_), std::bind(&ProdLineCtrl::packaging_info_handler, this, _1, _2));
    httpsvr_->Post(from_url(order_completion_), std::bind(&ProdLineCtrl::order_completion_handler, this, _1, _2, _3));
    httpsvr_->Get(from_url(cleaning_mac_scan_), std::bind(&ProdLineCtrl::cleaning_machine_scanner_handler, this, _1, _2));

    httpsvr_thread_ = std::thread(std::bind(&ProdLineCtrl::start_http_server, this)); 
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "init_httpsvr: %s", e.what());
    return false;
  }
  catch(...)
  {
    RCLCPP_ERROR(this->get_logger(), "init_httpsvr unknown error");
    return false;
  }
  
  return true;
}

void ProdLineCtrl::health_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;
  res.set_content("OK", "text/plain");
}

void ProdLineCtrl::abnormal_dispensation_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"},
    {"instructCode", 0}
  };

  if (!req.is_multipart_form_data()) 
  {
    std::string req_body;
    ctx_reader([&](const char *data, size_t data_length) {
      req_body.append(data, data_length);
      return true;
    });

    try 
    {
      nlohmann::json req_json = nlohmann::json::parse(req_body);
      DispensingError msg;
      msg.order_id = req_json["orderId"];
      msg.error_msg = req_json["errorMsg"];
      dis_err_pub_->publish(msg);

      res_json["code"] = 200;
      res_json["msg"] = "success";
      res_json["instructCode"] = 200; // FIXME

      RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
    } 
    catch (const std::exception &e) 
    {
      res_json["msg"] = "JSON parsing error";
    }
  } 
  
  std::string res_body = res_json.dump();
  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::abnormal_device_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"},
    {"instructCode", 0}
  };

  if (!req.is_multipart_form_data()) 
  {
    std::string req_body;
    ctx_reader([&](const char *data, size_t data_length) {
      req_body.append(data, data_length);
      return true;
    });

    try 
    {
      nlohmann::json req_json = nlohmann::json::parse(req_body);
      // TODO: Error msg

      res_json["code"] = 200;
      res_json["msg"] = "success";
      res_json["instructCode"] = 200;

      RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
    } 
    catch (const std::exception &e) 
    {
      res_json["msg"] = "JSON parsing error";
    }
  } 
  
  std::string res_body = res_json.dump();
  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::dispense_request_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"}
  };

  if (!req.is_multipart_form_data()) 
  {
    std::string req_body;
    ctx_reader([&](const char *data, size_t data_length) {
      req_body.append(data, data_length);
      return true;
    });

    try 
    {
      nlohmann::json req_json = nlohmann::json::parse(req_body);

      DispensingDetail msg;
      msg.amount = req_json["amount"];
      msg.location.dispenser_station = req_json["dispenserStation"];
      msg.location.dispenser_unit = req_json["dispenserUnit"];
      dis_req_pub_->publish(msg);

      res_json["code"] = 200;
      res_json["msg"] = "success";

      RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
    } 
    catch (const std::exception &e) 
    {
      res_json["msg"] = "JSON parsing error";
    }
  } 

  std::string res_body = res_json.dump();
  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::packaging_info_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"},
    {"packagingMachines", nlohmann::json::array()}
  };

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  lock.lock();
  for (const auto& x : pkg_mac_status) 
  {
    auto &x_val = x.second;

    nlohmann::json status_json;
    status_json["id"] = x_val.packaging_machine_id;
    status_json["state"] = x_val.packaging_machine_state;
    status_json["conveyorState"] = x_val.conveyor_state;
    status_json["canopenState"] = x_val.canopen_state;
    res_json["packagingMachines"].push_back(status_json);
  }
  lock.unlock();
  
  std::string res_body = res_json.dump();
  res_json["code"] = 200;
  res_json["msg"] = "success";

  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::cleaning_machine_scanner_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"}
  };

  if (req.has_param("materialBoxId")) 
  {
    unsigned int _id = stoi(req.get_param_value("materialBoxId"));
    if (_id < 255)
    {
      CleaningMachineScanner msg;
      msg.material_box_id = static_cast<uint8_t>(_id);
      msg.header.stamp = this->get_clock()->now();
      cleaning_mac_scan_pub_->publish(msg);

      res_json["code"] = 200;
      res_json["msg"] = "success";
    }
    else
    {
      res_json["msg"] = "material box id error. (id > 255)";
    }
  }
  else
  {
    res_json["msg"] = "Parameter: materialBoxId is missing";
  }

  std::string res_body = res_json.dump();
  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::order_completion_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    {"code", 0},
    {"msg", "failure"}
  };

  if (!req.is_multipart_form_data()) 
  {
    std::string req_body;
    ctx_reader([&](const char *data, size_t data_length) {
      req_body.append(data, data_length);
      return true;
    });

    try 
    {
      nlohmann::json req_json = nlohmann::json::parse(req_body);
      auto orders = req_json["orders"];
      for (const auto &order : orders)
      {
        OrderCompletion msg;
        msg.header.stamp = this->get_clock()->now();
        msg.material_box_id = order["materialBoxId"];
        // msg.order_id = order["orderId"];
        order_compl_pub_->publish(msg);
      }

      res_json["code"] = 200;
      res_json["msg"] = "success";
      RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
    } 
    catch (const std::exception &e) 
    {
      res_json["msg"] = "JSON parsing error";
    }
  } 

  std::string res_body = res_json.dump();
  res.set_content_provider(
    res_body.size(), 
    "application/json",
    [&, res_body](size_t offset, size_t length, httplib::DataSink &sink) {
      const auto &d = res_body;
      sink.write(&d[offset], std::min(length, DATA_CHUNK_SIZE));
      return true;
    });
}

void ProdLineCtrl::logger_handler(const httplib::Request &req, const httplib::Response &res)
{
  std::string s;
  char buf[BUFSIZ];

  s += "================================\n";
  snprintf(buf, sizeof(buf), "%s %s %s", req.method.c_str(),
           req.version.c_str(), req.path.c_str());
  s += buf;
  std::string query;
  for (auto it = req.params.begin(); it != req.params.end(); ++it) 
  {
    const auto &x = *it;
    snprintf(buf, sizeof(buf), "%c%s=%s",
             (it == req.params.begin()) ? '?' : '&', x.first.c_str(),
             x.second.c_str());
    query += buf;
  }
  snprintf(buf, sizeof(buf), "%s\n", query.c_str());
  s += buf;
  s += dump_headers(req.headers);
  s += dump_multipart_files(req.files);
  s += "--------------------------------\n";
  snprintf(buf, sizeof(buf), "%d\n", res.status);
  s += buf;
  s += dump_headers(res.headers);

  RCLCPP_DEBUG(this->get_logger(), "\n%s", s.c_str());
}

void ProdLineCtrl::error_handler(const httplib::Request &req, httplib::Response &res)
{
  (void)req;
  auto fmt = "<p>Error Status: <span style='color:red;'>%d</span></p>";
  char buf[BUFSIZ];
  snprintf(buf, sizeof(buf), fmt, res.status);
  res.set_content(buf, "text/html");
}

void ProdLineCtrl::exception_handler(const httplib::Request &req, httplib::Response &res, std::exception_ptr ep)
{
  (void)req;
  auto fmt = "<h1>Error 500</h1><p>%s</p>";
  char buf[BUFSIZ];

  try 
  {
    std::rethrow_exception(ep);
  } 
  catch (std::exception &e) 
  {
    snprintf(buf, sizeof(buf), fmt, e.what());
  } 
  catch (...) 
  { 
    snprintf(buf, sizeof(buf), fmt, "Unknown Exception");
  }

  res.set_content(buf, "text/html");
  res.status = httplib::StatusCode::InternalServerError_500;
}

httplib::Server::HandlerResponse ProdLineCtrl::pre_routing_handler(const httplib::Request &req, httplib::Response &res)
{
  (void)res;
  if (req.path == from_url("")) 
    return httplib::Server::HandlerResponse::Handled;

  return httplib::Server::HandlerResponse::Unhandled;
}

void ProdLineCtrl::start_http_server(void)
{
  try 
  { 
    svr_started_.store(true);
    RCLCPP_INFO(this->get_logger(), "HTTP server thread started");
    if (!httpsvr_->listen(httpsvr_ip_, httpsvr_port_)) 
    {
      RCLCPP_ERROR(this->get_logger(), "Server stopped in error state\n");
      rclcpp::shutdown();
    }
  }
  catch (...) 
  { 
    RCLCPP_ERROR(this->get_logger(), "HTTP server thread exception. Aborting."); 
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ProdLineCtrl>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}

// inline const std::string ProdLineCtrl::from_jinli_url(const std::string resource)
// {
//   return jinli_protocol_ + "://" + jinli_ip_ + ":" + jinli_port_ + resource;
// }