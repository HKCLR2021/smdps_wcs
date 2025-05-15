#include "dis_station/prod_line_ctrl.hpp"

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
    httpsvr_->Post(from_url(abnormal_dispensation), std::bind(&ProdLineCtrl::abnormal_dis_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(abnormal_device), std::bind(&ProdLineCtrl::abnormal_device_handler, this, _1, _2, _3));
    httpsvr_->Post(from_url(dispense_request), std::bind(&ProdLineCtrl::dis_req_handler, this, _1, _2, _3));
    httpsvr_->Get(from_url(packaging_request), std::bind(&ProdLineCtrl::pkg_req_handler, this, _1, _2));
    httpsvr_->Get(from_url(packaging_info), std::bind(&ProdLineCtrl::pkg_mac_info_handler, this, _1, _2));
    httpsvr_->Post(from_url(order_completion), std::bind(&ProdLineCtrl::order_comp_handler, this, _1, _2, _3));
    
    httpsvr_->Get(from_url(init_pkg_mac), std::bind(&ProdLineCtrl::init_pkg_mac_handler, this, _1, _2));

    httpsvr_->Get(from_url(scanner + cleaning_mac_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, cleaning_mac_loc));
    httpsvr_->Get(from_url(scanner + mtrl_box_con_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, mtrl_box_con_loc));
    httpsvr_->Get(from_url(scanner + pkg_mac_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, pkg_mac_loc));
    httpsvr_->Get(from_url(scanner + vis_inps_sys_scan), std::bind(&ProdLineCtrl::scanner_handler, this, _1, _2, vis_inps_sys_loc));
    
    httpsvr_->Get(from_url(con_ready), std::bind(&ProdLineCtrl::con_ready_handler, this, _1, _2));
    httpsvr_->Get(from_url(vis_stopper), std::bind(&ProdLineCtrl::vis_stopper_handler, this, _1, _2));

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

const std::string ProdLineCtrl::from_url(const std::string resource)
{
  return api + ver + resource;
}

bool ProdLineCtrl::verify_params(const httplib::Request &req, const std::vector<std::string> &keys) {
  return std::all_of(keys.begin(), keys.end(),
    [&req](auto p) {
      return req.has_param(p);
    }
  );
}

bool ProdLineCtrl::is_number(const std::string &s) 
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

size_t ProdLineCtrl::map_index(size_t index) 
{
  return (index / 4) + (index % 4) * 7;
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
  res_json["code"] = 0;
  res_json["msg"] = "failure";
  res_json["errorStatus"] = res.status;

  res.set_content(res_json.dump(), "application/json");
}

void ProdLineCtrl::exception_handler(const httplib::Request &req, httplib::Response &res, std::exception_ptr ep)
{
  (void)req;

  nlohmann::json res_json;
  res_json["code"] = 0;
  res_json["msg"] = "failure";

  try 
  {
    std::rethrow_exception(ep);
  } 
  catch (std::exception &e) 
  {
    res_json["exception"] = e.what();
  } 
  catch (...) 
  { 
    res_json["exception"] = "Unknown Exception";
  }

  res.set_content(res_json.dump(), "application/json");
  res.status = httplib::StatusCode::InternalServerError_500;
}

inline httplib::Server::HandlerResponse ProdLineCtrl::pre_routing_handler(const httplib::Request &req, httplib::Response &res)
{
  (void)res;

  if (req.path == from_url("")) 
    return httplib::Server::HandlerResponse::Handled;

  return httplib::Server::HandlerResponse::Unhandled;
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
