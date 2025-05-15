#ifndef DIS_STATION_NODE__
#define DIS_STATION_NODE__

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include <open62541pp/open62541pp.hpp>

#include "std_srvs/srv/trigger.hpp"

#include "smdps_msgs/msg/dispense_content.hpp"
#include "smdps_msgs/msg/dispense_result.hpp"
#include "smdps_msgs/msg/dispenser_station_status.hpp"
#include "smdps_msgs/msg/dispenser_unit_status.hpp"
#include "smdps_msgs/msg/unit_type.hpp"

#include "smdps_msgs/srv/dispense_drug.hpp"
#include "smdps_msgs/srv/unit_request.hpp"

#define NO_OF_UNITS 12
#define MAX_HB_COUNT 5

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class CallbackSignal
{
public:
  std::mutex cv_mutex_;
  std::condition_variable cv_;
  bool is_triggered_ = false;
};

class ValCallbackSignal : public CallbackSignal
{
public:
  int16_t val;
};

class DispenserStationNode : public rclcpp::Node 
{
  using Trigger = std_srvs::srv::Trigger;

  using DispenseContent = smdps_msgs::msg::DispenseContent;
  using DispenseResult = smdps_msgs::msg::DispenseResult;
  using DispenserStationStatus = smdps_msgs::msg::DispenserStationStatus;
  using DispenserUnitStatus = smdps_msgs::msg::DispenserUnitStatus;
  using DispenseDrug = smdps_msgs::srv::DispenseDrug;
  using UnitRequest = smdps_msgs::srv::UnitRequest;
  using UnitType = smdps_msgs::msg::UnitType;

public:
  explicit DispenserStationNode(const rclcpp::NodeOptions& options);
  ~DispenserStationNode();

  const std::string form_opcua_url(void);
  bool init_opcua_cli(void);
  void start_opcua_cli(void); 
  void wait_for_opcua_connection(const std::chrono::milliseconds freq);

  constexpr std::string_view get_enum_name(opcua::NodeClass node_class);
  constexpr std::string_view get_log_level_name(opcua::LogLevel level);
  constexpr std::string_view get_log_category_name(opcua::LogCategory category);
  void logger_wrapper(opcua::LogLevel level, opcua::LogCategory category, std::string_view msg);

  void disconnected_cb(void);
  void connected_cb(void);
  void session_activated_cb(void);
  void session_closed_cb(void);
  void inactive_cb(void);

  void create_sub_async(void);
  void create_mon_item_async(
    const opcua::CreateSubscriptionResponse &res, 
    const opcua::NodeId &id, 
    const std::string name, 
    std::shared_ptr<bool> ptr);
  void sub_status_change_cb(uint32_t sub_id, opcua::StatusChangeNotification &notification);
  void sub_deleted_cb(uint32_t sub_id);
  void monitored_item_deleted_cb(uint32_t sub_id, uint32_t mon_id, std::string name);
  void monitored_item_created_cb(opcua::MonitoredItemCreateResult &result, std::string name);

  void heartbeat_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void general_bool_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value, const std::string name, std::shared_ptr<bool> ptr);
  void alm_code_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void completed_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void dispensing_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void initiate_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void reset_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void cmd_amt_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value);
  void open_close_req_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value, const opcua::NodeId req_node_id, const opcua::NodeId state_node_id);
  
  void reset(void);
  void test_bin(void);
  void test_baffle(void);
  bool wait_for_futures(std::vector<std::future<opcua::StatusCode>> &futures);

  template <typename T>
  bool write_opcua_value(const opcua::NodeId& node_id, T value);
  template <typename T>
  bool read_opcua_value(const opcua::NodeId& node_id, std::shared_ptr<T> value);

  void dis_station_status_cb(void);
  void heartbeat_valid_cb(void);
  void units_lack_cb(void);

  void initiate(void);
  void clear_cmd_req(void);
  void clear_req(const opcua::NodeId req_node_id, const opcua::NodeId state_node_id);

  void dis_req_handle(
    const std::shared_ptr<DispenseDrug::Request> req, 
    std::shared_ptr<DispenseDrug::Response> res);
  void unit_req_handle(
    const std::shared_ptr<UnitRequest::Request> req, 
    std::shared_ptr<UnitRequest::Response> res);
  void init_bin_handle(
    const std::shared_ptr<Trigger::Request> req, 
    std::shared_ptr<Trigger::Response> res);
  void init_baffle_handle(
    const std::shared_ptr<Trigger::Request> req, 
    std::shared_ptr<Trigger::Response> res);
  void reset_handle(
    const std::shared_ptr<Trigger::Request> req, 
    std::shared_ptr<Trigger::Response> res);
  void init_handle(
    const std::shared_ptr<Trigger::Request> req, 
    std::shared_ptr<Trigger::Response> res);    
  void restart_handle(
    const std::shared_ptr<Trigger::Request> req, 
    std::shared_ptr<Trigger::Response> res);    

private:
  std::mutex mutex_;
  CallbackSignal com_signal;
  CallbackSignal init_signal;
  CallbackSignal reset_signal;
  ValCallbackSignal cmd_amt_signal;

  std::string ip_;
  std::string port_;

  rclcpp::CallbackGroup::SharedPtr heartbeat_cbg_;
  rclcpp::CallbackGroup::SharedPtr status_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr restart_cbg_;
  rclcpp::CallbackGroup::SharedPtr ree_srv_ser_cbg_;

  uint32_t heartbeat_counter_ = 0;
  const uint32_t OPCUA_TIMEOUT = 5000; // 5000ms

  std::shared_ptr<DispenserStationStatus> status_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr opcua_heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr units_lack_timer_;
  rclcpp::Publisher<DispenserStationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<DispenseResult>::SharedPtr dis_result_pub_;

  rclcpp::Subscription<DispenseContent>::SharedPtr dis_ctx_sub_;

  rclcpp::Service<DispenseDrug>::SharedPtr dis_req_srv_;

  rclcpp::Service<UnitRequest>::SharedPtr bin_req_srv_;
  rclcpp::Service<UnitRequest>::SharedPtr baffle_req_srv_;
  rclcpp::Service<Trigger>::SharedPtr init_bin_srv_;
  rclcpp::Service<Trigger>::SharedPtr init_baffle_srv_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<Trigger>::SharedPtr init_srv_;
  rclcpp::Service<Trigger>::SharedPtr restart_srv_;

protected:
  bool sim_;

  opcua::Client cli;
  std::atomic<bool> cli_started_;
  std::thread cli_thread_;

  const opcua::NamespaceIndex ns_ind = 4;
  const std::string send_prefix = "SEND|";
  const std::string rev_prefix = "REV|";

  const opcua::NodeId heartbeat_id        = {ns_ind, send_prefix + "Heartbeat"};
  const opcua::NodeId running_id          = {ns_ind, send_prefix + "Running"};
  const opcua::NodeId paused_id           = {ns_ind, send_prefix + "Paused"};
  const opcua::NodeId error_id            = {ns_ind, send_prefix + "Error"};
  const opcua::NodeId alm_code_id         = {ns_ind, send_prefix + "ALMCode"};

  const opcua::NodeId cmd_valid_state_id  = {ns_ind, send_prefix + "CmdValidState"};
  const opcua::NodeId cmd_amt_id          = {ns_ind, send_prefix + "CmdAmount"};
  const opcua::NodeId dispensing_id       = {ns_ind, send_prefix + "Dispensing"};
  const opcua::NodeId amt_dis_id          = {ns_ind, send_prefix + "AmountDispensed"};
  const opcua::NodeId completed_id        = {ns_ind, send_prefix + "Completed"};

  const opcua::NodeId initiate_id         = {ns_ind, rev_prefix + "Initiate"};

  const opcua::NodeId cmd_req_id          = {ns_ind, rev_prefix + "CmdRequest"};
  const opcua::NodeId cmd_exe_id          = {ns_ind, rev_prefix + "CmdExecute"};

  const opcua::NodeId reset_id            = {ns_ind, rev_prefix + "reset"};

  std::map<uint8_t, opcua::NodeId> unit_amt_id;
  
  std::map<uint8_t, opcua::NodeId> bin_open_req_id;
  std::map<uint8_t, opcua::NodeId> bin_close_req_id;
  std::map<uint8_t, opcua::NodeId> baffle_open_req_id;
  std::map<uint8_t, opcua::NodeId> baffle_close_req_id;

  std::map<uint8_t, opcua::NodeId> unit_lack_id;

  std::map<uint8_t, opcua::NodeId> bin_opened_id;
  std::map<uint8_t, opcua::NodeId> bin_closed_id;
  std::map<uint8_t, opcua::NodeId> baffle_opened_id;
  std::map<uint8_t, opcua::NodeId> baffle_closed_id;
};
#endif