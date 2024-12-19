#ifndef DIS_STATION_NODE__
#define DIS_STATION_NODE__

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <open62541pp/open62541pp.hpp>

#include "smdps_msgs/msg/dispenser_station_status.hpp"
#include "smdps_msgs/msg/dispenser_unit_status.hpp"
#include "smdps_msgs/srv/dispense_drug.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class DispenserStationNode : public rclcpp::Node
{
  using DispenserStationStatus = smdps_msgs::msg::DispenserStationStatus;
  using DispenserUnitStatus = smdps_msgs::msg::DispenserUnitStatus;
  using DispenseDrug = smdps_msgs::srv::DispenseDrug;

public:
  explicit DispenserStationNode(const rclcpp::NodeOptions& options);
  ~DispenserStationNode();

  inline const std::string form_opcua_url(void);
  bool init_opcua_cli(void);
  void start_opcua_cli(void); 

  constexpr std::string_view getLogLevelName(opcua::LogLevel level);
  constexpr std::string_view getLogCategoryName(opcua::LogCategory category);

  void disconnected_cb(void);
  void connected_cb(void);

private:
  std::mutex mutex_;
  std::string ip_;
  std::string port_;
  // bool is_connected_;

  std::shared_ptr<DispenserStationStatus> status_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Publisher<DispenserStationStatus>::SharedPtr status_pub_;

  rclcpp::Service<DispenseDrug>::SharedPtr dis_req_srv_;

  void dis_station_status_cb(void);
  void dis_req_handle(
    const std::shared_ptr<DispenseDrug::Request> req, 
    std::shared_ptr<DispenseDrug::Response> res);

protected:
  bool sim_;

  opcua::Client cli;
  std::atomic<bool> cli_started_;
  std::thread cli_thread_;
};
#endif