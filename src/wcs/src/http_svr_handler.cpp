#include "wcs/prod_line_ctrl.hpp"

void ProdLineCtrl::health_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;
  res.set_content("OK", "text/plain");
}

void ProdLineCtrl::abnormal_dis_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" },
    { "instructionCode", 0 }
  };

  if (req.is_multipart_form_data()) 
  {
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::string req_body;
  ctx_reader([&](const char *data, size_t data_length) {
    req_body.append(data, data_length);
    return true;
  });

  nlohmann::json req_json;
  try 
  {
    req_json = nlohmann::json::parse(req_body);
  }
  catch (const std::exception &e) 
  {
    res_json["msg"] = "JSON parsing error";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  DispensingError msg;
  msg.order_id = req_json["orderId"];
  msg.error_msg = req_json["errorMsg"];
  dis_err_pub_->publish(msg);

  res_json["code"] = 200;
  res_json["msg"] = "success";
  res_json["instructionCode"] = 200;

  RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::abnormal_device_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" },
    { "instructionCode", 0 }
  };

  if (req.is_multipart_form_data()) 
  {
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::string req_body;
  ctx_reader([&](const char *data, size_t data_length) {
    req_body.append(data, data_length);
    return true;
  });

  nlohmann::json req_json;
  try 
  {
    req_json = nlohmann::json::parse(req_body);
    // TODO: Error msg
  } 
  catch (const std::exception &e) 
  {
    res_json["msg"] = "JSON parsing error";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  res_json["code"] = 200;
  res_json["msg"] = "success";
  res_json["instructionCode"] = 0;

  RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
  
  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::dis_req_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (req.is_multipart_form_data()) 
  {
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::string req_body;
  ctx_reader([&](const char *data, size_t data_length) {
    req_body.append(data, data_length);
    return true;
  });

  nlohmann::json req_json;
  std::map<uint8_t, std::shared_ptr<DispenseDrug::Request>> dis_reqs;
  try
  {
    req_json = nlohmann::json::parse(req_body);
  } 
  catch (const std::exception &e) 
  {
    res_json["msg"] = "JSON parsing error";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n%s", req_body.c_str());

  for (const auto &loc : req_json["locations"])
  {
    uint8_t station_id = loc["dispenserStation"];
    if (dis_reqs.find(station_id) != dis_reqs.end()) 
    {
      auto &dis_req_ref = dis_reqs[station_id];
      DispenseContent msg;
      msg.unit_id = loc["dispenserUnit"];
      msg.amount = loc["amount"];
      dis_req_ref->content.push_back(msg);
    } 
    else 
    {
      std::shared_ptr<DispenseDrug::Request> request = std::make_shared<DispenseDrug::Request>();
      DispenseContent msg;
      msg.unit_id = loc["dispenserUnit"];
      msg.amount = loc["amount"];
      request->content.push_back(msg);
      dis_reqs[station_id] = request;
    }
  }

  RCLCPP_INFO(this->get_logger(), "%s, dis_reqs size: %ld", __FUNCTION__, dis_reqs.size());
  if (dis_reqs.size() > 0)
  {
    std::thread(&ProdLineCtrl::dis_result_srv_handler, this, std::move(dis_reqs)).detach();
  
    res_json["code"] = 200;
    res_json["msg"] = "success";
  }

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::pkg_req_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (req.is_multipart_form_data()) 
  {
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  bool pkg_mac_availability = false;
  lock.lock();
  for (const auto &status: pkg_mac_status_)
  {
    if (status.second.packaging_machine_state == PackagingMachineStatus::IDLE)
    {
      pkg_mac_availability = true;
      break;
    }
  }
  lock.unlock();

  if (!pkg_mac_availability)
  {
    res_json["msg"] = "Packaging Machines are busy";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  if (!req.has_param("materialBoxId"))
  {
    res_json["msg"] = "materialBoxId is missing";
    res.set_content(res_json.dump(), "application/json");
    return;
  } 
  
  const auto val = req.get_param_value("materialBoxId");

  if (!is_number(val))
  {
    res_json["msg"] = "materialBoxId is not a number";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const uint8_t id = static_cast<uint8_t>(stoi(val));

  std::thread pkg_srv_wait_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !pkg_order_cli_->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_ERROR(this->get_logger(), "PackagingOrder Service not available, waiting again...");
    }
  });

  std::shared_ptr<PackagingOrder::Request> pkg_order_srv_req = std::make_shared<PackagingOrder::Request>();
  pkg_order_srv_req->order_id = this->get_clock()->now().seconds();
  pkg_order_srv_req->material_box_id = id;
  pkg_order_srv_req->requester_id = 1234;

  for (size_t i = 0; i < pkg_order_srv_req->print_info.size(); ++i) 
  {
    if (std::get<0>(orders_[id]).material_box.slots[i].drugs.empty())
      continue;

    pkg_order_srv_req->print_info[i].cn_name = std::get<0>(orders_[id]).patient.institute_name;
    pkg_order_srv_req->print_info[i].en_name = std::get<0>(orders_[id]).patient.name;
    
    const uint8_t curr_meal = (std::get<0>(orders_[id]).start_meal + i) % 4;
    switch (curr_meal)
    {
    case OrderRequest::MEAL_MORNING:
      pkg_order_srv_req->print_info[i].time = "Morning";
      break;
    case OrderRequest::MEAL_NOON:
      pkg_order_srv_req->print_info[i].time = "Noon";
      break;
    case OrderRequest::MEAL_AFTERNOON:
      pkg_order_srv_req->print_info[i].time = "Afternoon";
      break;
    case OrderRequest::MEAL_EVENING:
      pkg_order_srv_req->print_info[i].time = "Evening";
      break;
    }
    
    const std::string date = std::get<0>(orders_[id]).start_date;
    std::tm t{};
    std::istringstream ss(date);

    ss >> std::get_time(&t, "%Y-%m-%d");
    if (ss.fail()) 
      throw std::runtime_error{"failed to parse time string"};
    
    std::time_t timestamp = mktime(&t);
    timestamp += ((std::get<0>(orders_[id]).start_meal + i) / 4) * 86400;

    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d", std::localtime(&timestamp));

    pkg_order_srv_req->print_info[i].date = "Date: " + std::string(mbstr); 
    pkg_order_srv_req->print_info[i].qr_code = "https://www.hkclr.hk";

    for (const auto &drug : std::get<0>(orders_[id]).material_box.slots[i].drugs)
    {
      const std::string drug_str = drug.name + "   " + std::to_string(drug.amount);
      RCLCPP_INFO(this->get_logger(), "%s", drug_str.c_str());
      pkg_order_srv_req->print_info[i].drugs.push_back(drug_str);
    }
  }

  using PkgServiceResponseFuture = rclcpp::Client<PackagingOrder>::SharedFuture;
  auto packaging_res_received_cb = [this, &res_json](PkgServiceResponseFuture future) {
    auto srv_res = future.get();
    if (srv_res && srv_res->success)
    {
      res_json["code"] = 200;
      res_json["msg"] = "success";
      RCLCPP_DEBUG(this->get_logger(), "Inside the PrintingOrder Callback OK");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the PrintingOrder Callback NOT OK. message: %s", srv_res->message.c_str());
    }
  };

  if (pkg_srv_wait_thread_.joinable())
    pkg_srv_wait_thread_.join();
  
  auto pkg_future = pkg_order_cli_->async_send_request(pkg_order_srv_req, packaging_res_received_cb);

  std::future_status pkg_status = pkg_future.wait_for(1s);
  switch (pkg_status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    res_json["msg"] = "pkg_future timeout";
    RCLCPP_ERROR(this->get_logger(), "PackagingOrder wait_for timeout");
    break;
  case std::future_status::deferred: 
    res_json["msg"] = "pkg_future deferred";
    RCLCPP_ERROR(this->get_logger(), "PackagingOrder wait_for deferred");
    break;
  }

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::pkg_mac_info_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;

  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" },
    { "packagingMachines", nlohmann::json::array() }
  };

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  lock.lock();
  for (const auto &status : pkg_mac_status_) 
  {
    const auto &status_val = status.second;

    nlohmann::json status_json;
    status_json["id"] = status_val.packaging_machine_id;
    status_json["state"] = status_val.packaging_machine_state;
    status_json["conveyorState"] = status_val.conveyor_state;
    status_json["canopenState"] = status_val.canopen_state;

    res_json["packagingMachines"].push_back(status_json);
  }
  lock.unlock();
  
  res_json["code"] = 200;
  res_json["msg"] = "success";

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::init_pkg_mac_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;

  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (!req.has_param("packagingMachineId")) 
  {
    res_json["msg"] = "packagingMachineId is missing";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const auto val = req.get_param_value("packagingMachineId");

  if (!is_number(val)) 
  {
    res_json["msg"] = "packagingMachineId is not a number";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const uint8_t pkg_mac_id = static_cast<uint8_t>(stoi(val));

  while (rclcpp::ok() && !init_pkg_mac_cli_[pkg_mac_id]->wait_for_service(std::chrono::seconds(1))) 
  {
    RCLCPP_ERROR(this->get_logger(), "Init Packaging Machine Service not available!");
  }

  std::shared_ptr<Trigger::Request> srv_req = std::make_shared<Trigger::Request>();

  using ServiceResponseFuture = rclcpp::Client<Trigger>::SharedFuture;
  
  auto res_received_cb = [this, &res_json](ServiceResponseFuture future) {
    auto srv_res = future.get();
    if (srv_res && srv_res->success)
    {
      res_json["code"] = 200;
      res_json["msg"] = "success";
      RCLCPP_DEBUG(this->get_logger(), "Inside the Init Packaging Machine Callback OK");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the Init Packaging Machine Callback NOT OK. message: %s", srv_res->message.c_str());
    }
  };
  
  auto future = init_pkg_mac_cli_[pkg_mac_id]->async_send_request(srv_req, res_received_cb);

  std::future_status status = future.wait_for(500ms);
  switch (status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "Init Packaging Machine wait_for timeout");
    break;
  case std::future_status::deferred: 
    RCLCPP_ERROR(this->get_logger(), "Init Packaging Machine wait_for deferred");
    break;
  }

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::scanner_handler(
  const httplib::Request &req, 
  httplib::Response &res,
  const std::string &location)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (!req.has_param("materialBoxId")) 
  {
    res_json["msg"] = "materialBoxId is missing";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const auto val = req.get_param_value("materialBoxId");
  if (!is_number(val)) 
  {
    res_json["msg"] = "materialBoxId is not a number";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const uint8_t mtrl_box_id = static_cast<uint8_t>(stoi(val));

  ScannerTrigger msg;
  msg.material_box_id = mtrl_box_id;
  msg.location = location;
  msg.header.stamp = this->get_clock()->now();
  scan_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Material box [%d] was passed through the location [%s]", mtrl_box_id, location.c_str());

  if (location == mtrl_box_con_loc)
  {
    if (con_mtrl_box_.empty() || con_mtrl_box_.front() != mtrl_box_id)
    {
      con_mtrl_box_.push(mtrl_box_id); // how to simpify?
      RCLCPP_INFO(this->get_logger(), "Added a material box [%d] to queue", mtrl_box_id);
    }   
  }
  else if (location == pkg_mac_loc)
  {
    std::thread srv_wait_thread = std::thread([this]() {
      while (rclcpp::ok() && !income_mtrl_box_cli_->wait_for_service(std::chrono::seconds(1))) 
      {
        RCLCPP_ERROR(this->get_logger(), "income_materal_box Service not available, waiting again...");
      }
    });
  
    std::shared_ptr<UInt8Srv::Request> srv_req = std::make_shared<UInt8Srv::Request>();
    srv_req->data = mtrl_box_id;
  
    using ServiceResponseFuture = rclcpp::Client<UInt8Srv>::SharedFuture;
    auto res_received_cb = [this](ServiceResponseFuture future) {
      auto srv_res = future.get();
      if (srv_res && srv_res->success)
        RCLCPP_DEBUG(this->get_logger(), "Inside the income_mtrl_box_cli_ Callback OK");
      else
        RCLCPP_ERROR(this->get_logger(), "Inside the income_mtrl_box_cli_ Callback NOT OK. message: %s", srv_res->message.c_str());
    };
  
    if (srv_wait_thread.joinable())
      srv_wait_thread.join();
  
    auto future = income_mtrl_box_cli_->async_send_request(srv_req, res_received_cb);
  
    std::future_status status = future.wait_for(1s);
    switch (status)
    {
    case std::future_status::ready:
      break;
    case std::future_status::timeout:
      RCLCPP_ERROR(this->get_logger(), "income_mtrl_box_cli_ wait_for timeout");
      res_json["msg"] = "timeout";
      res.set_content(res_json.dump(), "application/json");
      return;
    case std::future_status::deferred: 
      RCLCPP_ERROR(this->get_logger(), "income_mtrl_box_cli_ wait_for deferred");
      res_json["msg"] = "deferred";
      res.set_content(res_json.dump(), "application/json");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Added a material box id to income box queue");
  }

  res_json["code"] = 200;
  res_json["msg"] = "success";

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::con_ready_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;

  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  std::thread srv_wait_thread = std::thread([this]() {
    while (rclcpp::ok() && !container_ready_cli_->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_ERROR(this->get_logger(), "release_blocking Service not available, waiting again...");
    }
  });

  std::shared_ptr<Trigger::Request> srv_req = std::make_shared<Trigger::Request>();

  using ServiceResponseFuture = rclcpp::Client<Trigger>::SharedFuture;
  auto res_received_cb = [this, &res_json](ServiceResponseFuture future) {
    auto srv_res = future.get();
    if (srv_res && srv_res->success)
    {
      res_json["code"] = 200;
      res_json["msg"] = "success";
      RCLCPP_DEBUG(this->get_logger(), "Inside the release_blocking Callback OK");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the release_blocking Callback NOT OK. message: %s", srv_res->message.c_str());
    }
  };

  if (srv_wait_thread.joinable())
    srv_wait_thread.join();

  auto future = container_ready_cli_->async_send_request(srv_req, res_received_cb);

  std::future_status status = future.wait_for(1s);
  switch (status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "release_blocking wait_for timeout");
    res.set_content(res_json.dump(), "application/json");
    return;
  case std::future_status::deferred: 
    RCLCPP_ERROR(this->get_logger(), "release_blocking wait_for deferred");
    res.set_content(res_json.dump(), "application/json");
    return;
  }
  
  con_mtrl_box_.pop();
  RCLCPP_INFO(this->get_logger(), "Notified Manager to operate the conveyor and stopper");
  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::vis_stopper_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (!req.has_param("state")) 
  {
    res_json["msg"] = "state is missing";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  const auto val = req.get_param_value("state");
  if (!is_number(val)) 
  {
    res_json["msg"] = "state is not a number";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  // to notify vision system to do task

  res_json["code"] = 200;
  res_json["msg"] = "success";

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::order_comp_handler(
  const httplib::Request &req, 
  httplib::Response &res, 
  const httplib::ContentReader &ctx_reader)
{
  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" }
  };

  if (req.is_multipart_form_data()) 
  {
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::string req_body;
  ctx_reader([&](const char *data, size_t data_length) {
    req_body.append(data, data_length);
    return true;
  });

  nlohmann::json req_json;
  try 
  {
    req_json = nlohmann::json::parse(req_body);
  } 
  catch (const std::exception &e) 
  {
    res_json["msg"] = "JSON parsing error";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

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

  res.set_content(res_json.dump(), "application/json");
}

