#include "wcs/prod_line_ctrl.hpp"

bool ProdLineCtrl::init_httpcli(void)
{
  try
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    
    httpcli_ = std::make_shared<httplib::Client>(jinli_ip_, jinli_port_);
    httpcli_->set_connection_timeout(0, 1000 * 3000); // 3000ms 
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "init_httpcli: %s", e.what());
    return false;
  }
  catch(...)
  {
    RCLCPP_ERROR(this->get_logger(), "init_httpcli unknown error");
    return false;
  }
  
  return true;
}

bool ProdLineCtrl::get_material_box_info(nlohmann::json &res_json)
{
  auto res = httpcli_->Get(mtrl_box_info_url);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::get_material_box_info_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = httpcli_->Get(mtrl_box_info_by_id_url, params, headers);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::get_cells_info_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = httpcli_->Get(cells_info_by_id_url, params, headers);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::get_material_box_amt(nlohmann::json &res_json)
{
  auto res = httpcli_->Get(mtrl_box_amt_url);
  
  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::new_order(const nlohmann::json &req_json, nlohmann::json &res_json)
{
  httplib::Headers headers = {
    { "Content-Type", "application/json" }
  };
  std::string req_body = req_json.dump();

  auto res = httpcli_->Post(new_order_url, headers, req_body, "application/json");

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::get_order_by_id(const httplib::Params &params, nlohmann::json &res_json)
{
  httplib::Headers headers = {
    { "Content-Type", "text/plain" }
  };

  auto res = httpcli_->Get(order_by_id_url, params, headers);

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 
  
  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::dispense_result(const nlohmann::json &req_json, nlohmann::json &res_json)
{
  httplib::Headers headers = {
    { "Content-Type", "application/json" }
  };
  std::string req_body = req_json.dump();

  auto res = httpcli_->Post(dis_result_url, headers, req_body, "application/json");

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_INFO(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}

bool ProdLineCtrl::health_check(nlohmann::json &res_json)
{
  auto res = httpcli_->Get(health_url);

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  std::string msg = std::string(__FUNCTION__) + " error occurred. Error code: " + to_string(res.error());
  res_json["msg"] = msg;
  RCLCPP_ERROR(this->get_logger(), msg.c_str());
  return false;
}