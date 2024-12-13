#ifndef PROD_LINE_CTRL__
#define PROD_LINE_CTRL__

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "wcs/prod_line_ctrl.hpp"
#include "httplib/httplib.h"
#include "nlohmann/json.hpp"

#include "smdps_msgs/action/new_order.hpp"
#include "smdps_msgs/msg/cleaning_machine_scanner.hpp"
#include "smdps_msgs/msg/container_info.hpp"
#include "smdps_msgs/msg/dispensing_error.hpp"
#include "smdps_msgs/msg/dispensing_detail.hpp"
#include "smdps_msgs/msg/material_box_slot.hpp"
#include "smdps_msgs/msg/material_box_status.hpp"
#include "smdps_msgs/msg/material_box.hpp"
#include "smdps_msgs/msg/order_completion.hpp"
#include "smdps_msgs/msg/order_request.hpp"
#include "smdps_msgs/msg/order_response.hpp"
#include "smdps_msgs/msg/packaging_machine_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ProdLineCtrl : public rclcpp::Node
{
  using UInt8 = std_msgs::msg::UInt8;

  using NewOrder = smdps_msgs::action::NewOrder;
  using CleaningMachineScanner = smdps_msgs::msg::CleaningMachineScanner;
  using ContainerInfo = smdps_msgs::msg::ContainerInfo;
  using DispensingError = smdps_msgs::msg::DispensingError;
  using DispensingDetail = smdps_msgs::msg::DispensingDetail;
  using MaterialBoxSlot = smdps_msgs::msg::MaterialBoxSlot;
  using MaterialBoxStatus = smdps_msgs::msg::MaterialBoxStatus;
  using MaterialBox = smdps_msgs::msg::MaterialBox;
  using OrderCompletion = smdps_msgs::msg::OrderCompletion;
  using OrderRequest = smdps_msgs::msg::OrderRequest;
  using OrderResponse = smdps_msgs::msg::OrderResponse;
  using PackagingMachineStatus = smdps_msgs::msg::PackagingMachineStatus;

  using GaolHandlerNewOrder = rclcpp_action::ServerGoalHandle<NewOrder>;

public:
  explicit ProdLineCtrl(const rclcpp::NodeOptions& options);
  ~ProdLineCtrl();

  inline const std::string from_url(const std::string resource);
  inline const std::string from_jinli_url(const std::string resource);

  std::string dump_headers(const httplib::Headers &headers);
  std::string dump_multipart_files(const httplib::MultipartFormDataMap &files);
  inline bool verify_params(const httplib::Request &req, const std::vector<std::string> &keys);

private:
  std::mutex mutex_;
  std::map<uint8_t, PackagingMachineStatus> pkg_mac_status;

  rclcpp::TimerBase::SharedPtr mtrl_box_amt_timer_;
  rclcpp::TimerBase::SharedPtr mtrl_box_info_timer_;

  rclcpp::Publisher<ContainerInfo>::SharedPtr mtrl_box_amt_pub_;
  rclcpp::Publisher<DispensingError>::SharedPtr dis_err_pub_;
  rclcpp::Publisher<DispensingDetail>::SharedPtr dis_req_pub_;
  rclcpp::Publisher<CleaningMachineScanner>::SharedPtr cleaning_mac_scan_pub_;
  rclcpp::Publisher<OrderCompletion>::SharedPtr order_compl_pub_;

  rclcpp::Subscription<PackagingMachineStatus>::SharedPtr pkg_mac_status_sub_;

  rclcpp_action::Server<NewOrder>::SharedPtr action_server_;

  void mtrl_box_amt_container_cb(void);
  void mtrl_box_info_cb(void);
  void pkg_mac_status_cb(const PackagingMachineStatus::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const NewOrder::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GaolHandlerNewOrder> goal_handle);
  void handle_accepted(const std::shared_ptr<GaolHandlerNewOrder> goal_handle);
  void order_execute(const std::shared_ptr<GaolHandlerNewOrder> goal_handle);

  // HTTP Server function
  const std::string httpsvr_ip_ = "127.0.0.1"; // FIXME
  const int httpsvr_port_ = 7070; // FIXME 
  const size_t DATA_CHUNK_SIZE = 4; // FIXME 

  const std::string api_ = "/api";
  const std::string version_ = "/v1";
  
  const std::string healthy_               = "/health";
  const std::string abnormal_dispensation_ = "/abnormalDispensation";
  const std::string abnormal_device_       = "/abnormalDevice";
  const std::string dispense_request_      = "/dispenseRequest";
  const std::string packaging_request_     = "/packagingRequest";
  const std::string packaging_info_        = "/packagingMachineInfo";
  const std::string order_completion_       = "/orderCompeletion";
  const std::string cleaning_mac_scan_     = "/cleaningMachineScanner";

  void healthy_handler(const httplib::Request &req, httplib::Response &res);
  void abnormal_dispensation_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader);
  void abnormal_device_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader);
  void dispense_request_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader);
  void packaging_request_handler(const httplib::Request &req, httplib::Response &res);
  void packaging_info_handler(const httplib::Request &req, httplib::Response &res);
  void order_completion_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader);
  void cleaning_machine_scanner_handler(const httplib::Request &req, httplib::Response &res);
  
  void logger_handler(const httplib::Request& req, const httplib::Response& res);
  void error_handler(const httplib::Request &req, httplib::Response &res);
  void exception_handler(const httplib::Request &req, httplib::Response &res, std::exception_ptr ep);
  httplib::Server::HandlerResponse pre_routing_handler(const httplib::Request &req, httplib::Response &res);
  void start_http_server(void);

  // HTTP Client function
  const std::string jinli_protocol_ = "http"; // FIXME
  const std::string jinli_ip_ = "192.168.0.110";  // FIXME
  const int jinli_port_ = 8080; // FIXME

  const std::string mtrl_box_info_url_       = "/sort/materialBox/getMaterialBoxInfo";
  const std::string mtrl_box_info_by_id_url_ = "/sort/materialBox/getMaterialBoxInfoById";
  const std::string cells_info_by_id_url_    = "/sort/cell/CellInfoByMaterialBoxId";
  const std::string mtrl_box_amt_url_        = "/sort/container/materialBoxAmount";
  const std::string new_order_url_           = "/sort/order/newOrder";
  const std::string order_by_id_url_         = "/sort/order/getOrderById";

  bool init_httpsvr(void);
  bool get_material_box_info(nlohmann::json &body_json);
  bool get_material_box_info_by_id(const httplib::Params &params, nlohmann::json &res_json);
  bool get_cell_info_by_id(const httplib::Params &params, nlohmann::json &res_json);
  bool get_material_box_amt(nlohmann::json &res_json);
  bool new_order(const nlohmann::json &req_json, nlohmann::json &res_json);
  bool get_order_by_id(const httplib::Params &params, nlohmann::json &res_json);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  size_t count_;

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    pub_->publish(message);
  }

protected:
  std::shared_ptr<httplib::Server> httpsvr_;
  std::atomic<bool> svr_started = std::atomic<bool>{false};
  std::thread httpsvr_thread_;
};

#endif // PROD_LINE_CTRL__