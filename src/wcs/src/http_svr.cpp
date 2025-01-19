#include "wcs/prod_line_ctrl.hpp"

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
    httpsvr_->set_payload_max_length(1024 * 1024 * 8); // 8MB

    httpsvr_->Get(from_url(health), std::bind(&ProdLineCtrl::health_handler, this, _1, _2));
    httpsvr_->Post(from_url(abnormal_dispensation), std::bind(&ProdLineCtrl::abnormal_dispensation_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(abnormal_device), std::bind(&ProdLineCtrl::abnormal_device_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(dispense_request), std::bind(&ProdLineCtrl::dispense_request_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(packaging_request), std::bind(&ProdLineCtrl::packaging_request_handler, this, _1, _2, _3));
    httpsvr_->Get(from_url(packaging_info), std::bind(&ProdLineCtrl::packaging_info_handler, this, _1, _2));
    httpsvr_->Post(from_url(order_completion), std::bind(&ProdLineCtrl::order_completion_handler, this, _1, _2, _3));

    httpsvr_->Get(from_url(scanner + cleaning_mac_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, cleaning_mac_loc));
    httpsvr_->Get(from_url(scanner + mtrl_box_con_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, mtrl_box_con_loc));
    httpsvr_->Get(from_url(scanner + pkg_mac_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, pkg_mac_loc));
    httpsvr_->Get(from_url(scanner + vis_inps_sys_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, vis_inps_sys_loc));

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

void ProdLineCtrl::start_http_server(void)
{
  svr_started_.store(true);

  while (svr_started_.load() && rclcpp::ok())
  {
    try 
    { 
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
    
    std::this_thread::sleep_for(1s);
  }
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
  res_json["instructionCode"] = 200; // FIXME

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

  try 
  {
    nlohmann::json req_json = nlohmann::json::parse(req_body);
    // TODO: Error msg

    res_json["code"] = 200;
    res_json["msg"] = "success";
    res_json["instructionCode"] = 200;

    RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());
  } 
  catch (const std::exception &e) 
  {
    res_json["msg"] = "JSON parsing error";
  }
  
  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::dispense_request_handler(
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
      dis_req_ref->content.push_back(std::move(msg));
    } 
    else 
    {
      std::shared_ptr<DispenseDrug::Request> request = std::make_shared<DispenseDrug::Request>();
      DispenseContent msg;
      msg.unit_id = loc["dispenserUnit"];
      msg.amount = loc["amount"];
      request->content.push_back(std::move(msg));
      dis_reqs[station_id] = request;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "dis_reqs size: %ld", dis_reqs.size());

  auto dis_reqs_ptr = std::make_unique<decltype(dis_reqs)>(dis_reqs);
  std::thread(std::bind(&ProdLineCtrl::dis_result_handler, this, *dis_reqs_ptr)).detach(); 

  res_json["code"] = 200;
  res_json["msg"] = "success";

  RCLCPP_DEBUG(this->get_logger(), "\n%s", req_body.c_str());

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::packaging_request_handler(
  const httplib::Request &req, 
  httplib::Response &res,
  const httplib::ContentReader &ctx_reader)
{
  (void)ctx_reader;

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
  for (const auto &status: pkg_mac_status)
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
    res_json["msg"] = "Packaging Machines are not idle";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  if (!req.has_param("materialBoxId"))
  {
    res_json["msg"] = "materialBoxId is not found";
    res.set_content(res_json.dump(), "application/json");
    return;
  } 
  
  if (!is_number(req.get_param_value("materialBoxId")))
  {
    res_json["msg"] = "materialBoxId is not a number";
    res.set_content(res_json.dump(), "application/json");
    return;
  }

  std::thread print_srv_wait_thread_ = std::thread([this]() {
    while (!printing_info_cli_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "PrintingOrder Service not available, waiting again...");
      std::this_thread::sleep_for(1s);
    }
  });

  const auto val = req.get_param_value("materialBoxId");
  const uint8_t id = static_cast<uint8_t>(stoi(val));

  std::shared_ptr<PrintingOrder::Request> printing_info_srv_req = std::make_shared<PrintingOrder::Request>();
  printing_info_srv_req->material_box_id = id;

  std::shared_ptr<PrintingInfo> info = std::make_shared<PrintingInfo>();

  using ServiceResponseFuture = rclcpp::Client<PrintingOrder>::SharedFuture;
  auto printing_res_received_cb = [this, info](ServiceResponseFuture future) {
    auto srv_res = future.get();
    if (srv_res && srv_res->success) 
    {
      *info = srv_res->info;
      RCLCPP_DEBUG(this->get_logger(), "Inside the PrintingOrder Callback OK");
    } else 
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the PrintingOrder Callback NOT OK. message: %s", srv_res->message.c_str());
    }
  };

  if (print_srv_wait_thread_.joinable())
    print_srv_wait_thread_.join();

  auto print_future = printing_info_cli_->async_send_request(printing_info_srv_req, printing_res_received_cb);

  std::future_status print_status = print_future.wait_for(500ms);
  switch (print_status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "PrintingOrder wait_for timeout");
    return;
  default: 
    RCLCPP_ERROR(this->get_logger(), "PrintingOrder wait_for NOT OK");
    return;
  }

  std::thread pkg_srv_wait_thread_ = std::thread([this]() {
    while (!pkg_order_cli_->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_ERROR(this->get_logger(), "PackagingOrder Service not available, waiting again...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  std::shared_ptr<PackagingOrder::Request> pkg_order_srv_req = std::make_shared<PackagingOrder::Request>();
  pkg_order_srv_req->order_id = 0;
  pkg_order_srv_req->material_box_id = id;
  pkg_order_srv_req->requester_id = 0;

  for (size_t i = 0; i < info->slots.size(); ++i) 
  {
    pkg_order_srv_req->print_info[i].cn_name = info->patient_info.institute_name;
    pkg_order_srv_req->print_info[i].en_name = info->patient_info.name;
    pkg_order_srv_req->print_info[i].date = "2023-11-27 (FIXME)";
    pkg_order_srv_req->print_info[i].time = "12:00 (FIXME)";
    pkg_order_srv_req->print_info[i].qr_code = info->qr_code;
    pkg_order_srv_req->print_info[i].drugs = {"FIXME 1", "FIXME 2"};
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

  std::future_status pkg_status = pkg_future.wait_for(500ms);
  switch (pkg_status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "PackagingOrder wait_for timeout");
    return;
  default: 
    RCLCPP_ERROR(this->get_logger(), "PackagingOrder wait_for NOT OK");
    return;
  }

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::packaging_info_handler(
  const httplib::Request &req, 
  httplib::Response &res)
{
  (void)req;

  nlohmann::json res_json = {
    { "code", 0 },
    { "msg", "failure" },
    {"packagingMachines", nlohmann::json::array()}
  };

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  lock.lock();
  for (const auto& status : pkg_mac_status) 
  {
    const auto &status_val = status.second;

    nlohmann::json status_json;
    status_json["id"] = status_val.packaging_machine_id;
    status_json["state"] = status_val.packaging_machine_state;
    status_json["conveyorState"] = status_val.conveyor_state;
    status_json["canopenState"] = status_val.canopen_state;

    res_json["packagingMachines"].push_back(std::move(status_json));
  }
  lock.unlock();
  
  res_json["code"] = 200;
  res_json["msg"] = "success";

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
    res_json["msg"] = "Parameter: materialBoxId is missing";
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
  cleaning_mac_scan_pub_->publish(msg);

  res_json["code"] = 200;
  res_json["msg"] = "success";

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::order_completion_handler(
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
  nlohmann::json res_json;

  res_json["errorStatus"] = res.status;

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::exception_handler(const httplib::Request &req, httplib::Response &res, std::exception_ptr ep)
{
  (void)req;
  nlohmann::json res_json;

  try 
  {
    std::rethrow_exception(ep);
  } 
  catch (std::exception &e) 
  {
    res_json["error"] = e.what();
  } 
  catch (...) 
  { 
    res_json["error"] = "Unknown Exception";
  }

  res.set_content(res_json.dump(), "application/json");
  res.status = httplib::StatusCode::InternalServerError_500;
}

httplib::Server::HandlerResponse ProdLineCtrl::pre_routing_handler(const httplib::Request &req, httplib::Response &res)
{
  (void)res;
  if (req.path == from_url("")) 
    return httplib::Server::HandlerResponse::Handled;

  return httplib::Server::HandlerResponse::Unhandled;
}

inline const std::string ProdLineCtrl::from_url(const std::string resource)
{
  return api + ver + resource;
}

inline bool ProdLineCtrl::verify_params(const httplib::Request &req, const std::vector<std::string> &keys) {
  return std::all_of(keys.begin(), keys.end(),
    [&req](auto p) {
      return req.has_param(p);
    }
  );
}

inline bool ProdLineCtrl::is_number(const std::string &s) 
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
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
