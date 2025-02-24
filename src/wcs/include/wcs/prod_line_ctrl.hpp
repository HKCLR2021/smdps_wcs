#ifndef PROD_LINE_CTRL__
#define PROD_LINE_CTRL__

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wcs/api_endpoints.hpp"
#include "wcs/prod_line_ctrl.hpp"
#include "httplib/httplib.h"
#include "nlohmann/json.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "smdps_msgs/action/new_order.hpp"

#include "smdps_msgs/msg/container_info.hpp"
#include "smdps_msgs/msg/dispense_content.hpp"
#include "smdps_msgs/msg/dispensing_error.hpp"
#include "smdps_msgs/msg/dispensing_detail.hpp"
#include "smdps_msgs/msg/heartbeat.hpp"
#include "smdps_msgs/msg/material_box_slot.hpp"
#include "smdps_msgs/msg/material_box_status.hpp"
#include "smdps_msgs/msg/material_box.hpp"
#include "smdps_msgs/msg/order_completion.hpp"
#include "smdps_msgs/msg/order_request.hpp"
#include "smdps_msgs/msg/order_response.hpp"
#include "smdps_msgs/msg/packaging_machine_status.hpp"
#include "smdps_msgs/msg/printing_info.hpp"
#include "smdps_msgs/msg/scanner_trigger.hpp"
#include "smdps_msgs/msg/unbind_request.hpp"

#include "smdps_msgs/srv/dispense_drug.hpp"
#include "smdps_msgs/srv/packaging_order.hpp"
#include "smdps_msgs/srv/printing_order.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ProdLineCtrl : public rclcpp::Node
{
  using UInt8 = std_msgs::msg::UInt8;
  using Trigger = std_srvs::srv::Trigger;

  using NewOrder = smdps_msgs::action::NewOrder;
  using ContainerInfo = smdps_msgs::msg::ContainerInfo;
  using DispenseContent = smdps_msgs::msg::DispenseContent;
  using DispensingError = smdps_msgs::msg::DispensingError;
  using DispensingDetail = smdps_msgs::msg::DispensingDetail;
  using Heartbeat = smdps_msgs::msg::Heartbeat;
  using MaterialBoxSlot = smdps_msgs::msg::MaterialBoxSlot;
  using MaterialBoxStatus = smdps_msgs::msg::MaterialBoxStatus;
  using MaterialBox = smdps_msgs::msg::MaterialBox;
  using OrderCompletion = smdps_msgs::msg::OrderCompletion;
  using OrderRequest = smdps_msgs::msg::OrderRequest;
  using OrderResponse = smdps_msgs::msg::OrderResponse;
  using PackagingMachineStatus = smdps_msgs::msg::PackagingMachineStatus;
  using PrintingInfo = smdps_msgs::msg::PrintingInfo;
  using ScannerTrigger = smdps_msgs::msg::ScannerTrigger;
  using UnbindRequest = smdps_msgs::msg::UnbindRequest;

  using GaolHandlerNewOrder = rclcpp_action::ServerGoalHandle<NewOrder>;

  using DispenseDrug = smdps_msgs::srv::DispenseDrug;
  using PackagingOrder = smdps_msgs::srv::PackagingOrder;
  using PrintingOrder = smdps_msgs::srv::PrintingOrder;

public:
  explicit ProdLineCtrl(const rclcpp::NodeOptions& options);
  ~ProdLineCtrl();

  inline const std::string from_url(const std::string resource);
  inline const std::string from_jinli_url(const std::string resource);

  std::string dump_headers(const httplib::Headers &headers);
  std::string dump_multipart_files(const httplib::MultipartFormDataMap &files);
  inline bool verify_params(const httplib::Request &req, const std::vector<std::string> &keys);
  inline bool is_number(const std::string &s);
  size_t map_index(size_t index);

private:
  std::mutex mutex_;
  std::map<uint8_t, PackagingMachineStatus> pkg_mac_status_;

  std::map<uint8_t, std::tuple<OrderRequest, uint8_t>> orders_;

  rclcpp::TimerBase::SharedPtr hc_timer_;
  rclcpp::TimerBase::SharedPtr mtrl_box_amt_timer_;
  rclcpp::TimerBase::SharedPtr mtrl_box_info_timer_;

  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr hc_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr container_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr mtrl_box_info_timer_cbg_;

  rclcpp::Publisher<Heartbeat>::SharedPtr hc_pub_;
  rclcpp::Publisher<ContainerInfo>::SharedPtr mtrl_box_amt_pub_;
  rclcpp::Publisher<DispensingError>::SharedPtr dis_err_pub_;
  rclcpp::Publisher<ScannerTrigger>::SharedPtr scan_pub_;
  rclcpp::Publisher<OrderCompletion>::SharedPtr order_compl_pub_;
  rclcpp::Publisher<MaterialBoxStatus>::SharedPtr mtrl_box_status_pub_;

  rclcpp::Subscription<PackagingMachineStatus>::SharedPtr pkg_mac_status_sub_;
  rclcpp::Subscription<UnbindRequest>::SharedPtr unbind_mtrl_id_sub_;

  rclcpp::Client<PrintingOrder>::SharedPtr printing_info_cli_;
  rclcpp::Client<PackagingOrder>::SharedPtr pkg_order_cli_;
  std::map<uint8_t, rclcpp::Client<Trigger>::SharedPtr> init_pkg_mac_cli_;
  std::map<uint8_t, rclcpp::Client<DispenseDrug>::SharedPtr> dis_req_cli_;

  rclcpp_action::Server<NewOrder>::SharedPtr action_server_;

  void hc_cb(void);
  void mtrl_box_amt_container_cb(void);
  void mtrl_box_info_cb(void);
  void pkg_mac_status_cb(const PackagingMachineStatus::SharedPtr msg);
  void unbind_mtrl_id_cb(const UnbindRequest::SharedPtr msg);

  void dis_result_srv_handler(std::map<uint8_t, std::shared_ptr<DispenseDrug::Request>> dis_reqs);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const NewOrder::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GaolHandlerNewOrder> goal_handle);
  void handle_accepted(const std::shared_ptr<GaolHandlerNewOrder> goal_handle);
  void order_execute(const std::shared_ptr<GaolHandlerNewOrder> goal_handle);

  // HTTP Server function
  std::string httpsvr_ip_; // FIXME
  int httpsvr_port_; // FIXME 
  const size_t DATA_CHUNK_SIZE = 4; // FIXME 

  void health_handler(const httplib::Request &req, httplib::Response &res);
  void abnormal_dis_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &ctx_reader);
  void abnormal_device_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &ctx_reader);
  void dis_req_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &ctx_reader);
  void pkg_req_handler(const httplib::Request &req, httplib::Response &res);
  void pkg_mac_info_handler(const httplib::Request &req, httplib::Response &res);
  void order_comp_handler(const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &ctx_reader);
  void scanner_handler(const httplib::Request &req, httplib::Response &res, const std::string &location);
  void init_pkg_mac_handler(const httplib::Request &req, httplib::Response &res);

  // void dis_result_handler(std::vector<std::tuple<uint8_t, bool, ServiceSharedFutureAndRequestId>> futures);

  void logger_handler(const httplib::Request& req, const httplib::Response& res);
  void error_handler(const httplib::Request &req, httplib::Response &res);
  void exception_handler(const httplib::Request &req, httplib::Response &res, std::exception_ptr ep);
  httplib::Server::HandlerResponse pre_routing_handler(const httplib::Request &req, httplib::Response &res);
  void start_http_server(void);

  // HTTP Client function
  const std::string jinli_protocol_ = "http"; // FIXME
  std::string jinli_ip_;
  int jinli_port_;

  size_t no_of_dis_stations_;
  size_t no_of_pkg_mac_;

  bool init_httpsvr(void);
  bool init_httpcli(void);
  
  bool get_mtrl_box_info(nlohmann::json &body_json);
  bool get_mtrl_box_info_by_id(const httplib::Params &params, nlohmann::json &res_json);
  bool get_cells_info_by_id(const httplib::Params &params, nlohmann::json &res_json);
  bool get_cell_info_by_id_and_cell_id(const httplib::Params &params, nlohmann::json &res_json);
  bool get_mtrl_box_amt(nlohmann::json &res_json);
  bool new_order(const nlohmann::json &req_json, nlohmann::json &res_json);
  void new_order_until_success(const nlohmann::json &req_json, nlohmann::json &res_json);
  bool get_order_by_id(const httplib::Params &params, nlohmann::json &res_json);
  bool dis_result(const nlohmann::json &req_json, nlohmann::json &res_json);
  void dis_result_until_success(const nlohmann::json &req_json, nlohmann::json &res_json);
  bool health_check(nlohmann::json &res_json);

  void perform_until_success(
    const nlohmann::json &req_json, 
    nlohmann::json &res_json, 
    std::function<bool(const nlohmann::json&, nlohmann::json&)> func);

protected:
  std::shared_ptr<httplib::Server> httpsvr_;
  std::shared_ptr<httplib::Client> httpcli_;
  std::atomic<bool> svr_started_;
  bool jinli_ser_state_;
  std::thread httpsvr_thread_;
};

#endif // PROD_LINE_CTRL__