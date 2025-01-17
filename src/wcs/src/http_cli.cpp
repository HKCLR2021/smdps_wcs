#include "wcs/prod_line_ctrl.hpp"

bool ProdLineCtrl::get_material_box_info(nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 

  auto res = cli.Get(mtrl_box_info_url);
  
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

  auto res = cli.Get(mtrl_box_info_by_id_url, params, headers);
  
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

  auto res = cli.Get(cells_info_by_id_url, params, headers);
  
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

  auto res = cli.Get(mtrl_box_amt_url);
  
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

  auto res = cli.Post(new_order_url, headers, req_body, "application/json");

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

  auto res = cli.Get(order_by_id_url, params, headers);

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

  auto res = cli.Post(dis_result_url, headers, req_body, "application/json");

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_INFO(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}

bool ProdLineCtrl::health_check(nlohmann::json &res_json)
{
  httplib::Client cli(jinli_ip_, jinli_port_);
  cli.set_connection_timeout(0, 1000 * 1000); // 1000ms 
  
  auto res = cli.Get(health_url);

  if (res && res->status == httplib::StatusCode::OK_200) 
  {
    RCLCPP_DEBUG(this->get_logger(), "%s OK", __FUNCTION__);
    res_json = nlohmann::json::parse(res->body);
    return true;
  } 

  RCLCPP_ERROR(this->get_logger(), "%s error occurred. Error code: %s", __FUNCTION__, to_string(res.error()).c_str());
  return false;
}