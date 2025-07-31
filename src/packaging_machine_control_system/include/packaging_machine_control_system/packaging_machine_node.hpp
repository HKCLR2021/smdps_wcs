#ifndef PACKAGING_MACHINE_NODE_HPP_
#define PACKAGING_MACHINE_NODE_HPP_

#pragma once 

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <iostream>
#include <fstream>
#include <queue>
#include <cstdlib>
#include <math.h>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "smdps_msgs/action/packaging_order.hpp"

#include "smdps_msgs/msg/packaging_machine_status.hpp"
#include "smdps_msgs/msg/packaging_machine_info.hpp"
#include "smdps_msgs/msg/package_info.hpp"
#include "smdps_msgs/msg/motor_status.hpp"
#include "smdps_msgs/msg/unbind_request.hpp"

#include "smdps_msgs/srv/u_int8.hpp"
#include "smdps_msgs/srv/u_int32.hpp"

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

#include "printer/config.h"
#include "printer/printer.h"

#include "packaging_machine_definition.hpp"

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

class PackagingMachineNode : public rclcpp::Node
{
public:
  using Bool = std_msgs::msg::Bool;
  using UInt8 = std_msgs::msg::UInt8;

  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  using COData = canopen_interfaces::msg::COData;
  using CORead = canopen_interfaces::srv::CORead;
  using COWrite = canopen_interfaces::srv::COWrite;

  using PackagingMachineStatus = smdps_msgs::msg::PackagingMachineStatus;
  using PackagingMachineInfo = smdps_msgs::msg::PackagingMachineInfo;
  using PackageInfo = smdps_msgs::msg::PackageInfo;
  using MotorStatus = smdps_msgs::msg::MotorStatus;
  using UnbindRequest = smdps_msgs::msg::UnbindRequest;
  using PackagingOrder = smdps_msgs::action::PackagingOrder;

  using UInt8Srv = smdps_msgs::srv::UInt8;
  using UInt32Srv = smdps_msgs::srv::UInt32;

  using GaolHandlerPackagingOrder = rclcpp_action::ServerGoalHandle<PackagingOrder>;

  explicit PackagingMachineNode(const rclcpp::NodeOptions& options);
  ~PackagingMachineNode() = default;

  uint32_t read_ribbon(std::string type);
  void write_ribbon(std::string type, uint32_t ribbon_length);

  void pub_status_cb(void);
  void heater_cb(void);
  void con_state_cb(void);
  void remain_length_cb(void);
  void init_timer(void);

  void co_read_wait_for_service(void);
  void co_write_wait_for_service(void);

  template <uint16_t index>
  bool read_co(std::shared_ptr<uint32_t> data);
  std::optional<uint32_t> call_co_read(uint16_t index, uint8_t subindex);
  bool call_co_write(uint16_t index, uint8_t subindex, uint32_t data);

  bool ctrl_heater(const bool on); 
  bool write_heater(const uint32_t data); 
  bool read_heater(std::shared_ptr<uint32_t> data); 

  bool ctrl_stopper(const bool protrude); 
  bool write_stopper(const uint32_t data); 
  bool read_stopper(std::shared_ptr<uint32_t> data);
  
  bool ctrl_material_box_gate(const bool open); 
  bool write_material_box_gate(const uint32_t data); 
  bool read_material_box_gate(std::shared_ptr<uint32_t> data); 

  bool ctrl_cutter(const bool cut);
  bool write_cutter(const uint32_t data);
  bool read_cutter(std::shared_ptr<uint32_t> data);

  bool ctrl_pkg_dis(const float length, const bool feed, const bool ctrl);
  bool read_pkg_dis_state(std::shared_ptr<uint32_t> data);
  bool read_pkg_dis_ctrl(std::shared_ptr<uint32_t> data);

  bool ctrl_pill_gate(const float length, const bool open, const bool ctrl);
  bool read_pill_gate_state(std::shared_ptr<uint32_t> data);
  bool read_pill_gate_ctrl(std::shared_ptr<uint32_t> data);
  
  bool ctrl_squeezer(const bool squeeze, const bool ctrl);
  bool read_squeezer_state(std::shared_ptr<uint32_t> data);
  bool read_squeezer_ctrl(std::shared_ptr<uint32_t> data);

  bool ctrl_conveyor(const uint16_t speed, const bool stop_by_ph, const bool fwd, const bool ctrl);
  bool read_conveyor_state(std::shared_ptr<uint32_t> data);
  bool read_conveyor_ctrl(std::shared_ptr<uint32_t> data);

  bool ctrl_roller(const uint8_t days, const bool home, const bool ctrl);
  bool read_roller_state(std::shared_ptr<uint32_t> data);
  bool read_roller_ctrl(std::shared_ptr<uint32_t> data);
  
  bool ctrl_pkg_len(const uint8_t level, const bool ctrl);
  bool read_pkg_len_state(std::shared_ptr<uint32_t> data);
  bool read_pkg_len_ctrl(std::shared_ptr<uint32_t> data);

  void wait_for_stopper(const uint32_t stop_condition);
  void wait_for_material_box_gate(const uint32_t stop_condition);
  void wait_for_cutter(const uint32_t stop_condition);

  // template<bool state_reader(std::shared_ptr<uint32_t>), 
  //   bool ctrl_reader(std::shared_ptr<uint32_t>)>
  // void wait_for_motor_state(
  //   state_reader read_state_func,
  //   ctrl_reader read_ctrl_func,
  //   const uint8_t target_state,
  //   const std::string& motor_name);

  void wait_for_pkg_dis(const uint8_t target_state = MotorStatus::IDLE);
  void wait_for_pill_gate(const uint8_t target_state = MotorStatus::IDLE);
  void wait_for_squeezer(const uint8_t target_state = MotorStatus::IDLE);
  void wait_for_conveyor(const uint8_t target_state = MotorStatus::IDLE);
  void wait_for_roller(const uint8_t target_state = MotorStatus::IDLE);
  void wait_for_pkg_len(const uint8_t target_state = MotorStatus::IDLE);

  void init_printer(void);
  void init_printer_config(void);
  std::string add_drug_space(std::string drug_str);
  smdps_msgs::msg::PackageInfo create_printer_info_temp(void);
  std::vector<std::string> get_print_label_cmd(std::string name, int total, int current);
  std::vector<std::string> get_print_label_cmd(PackageInfo msg);

  void init_packaging_machine(void);

private:
  std::mutex mutex_;
  std::recursive_mutex  r_mutex_;

  std::shared_ptr<Printer> printer_;
  std::shared_ptr<Config> printer_config_;

  CallbackSignal co_read_signal;
  CallbackSignal co_write_signal;

  uint8_t printer_font_ = 0;
  uint8_t MAX_DRUG_LEN = 34;

  std::vector<std::string> printer_test_date;
  std::vector<std::string> printer_test_meal;
  uint8_t printer_test_date_index = 0;
  uint8_t printer_test_meal_index = 0;

  bool is_initialized_ = false;
  bool sim_;
  bool enable_heater_;
  std::shared_ptr<PackagingMachineStatus> status_;
  std::shared_ptr<MotorStatus> motor_status_;
  std::shared_ptr<PackagingMachineInfo> info_;

  rclcpp::CallbackGroup::SharedPtr co_cli_read_cbg_;
  rclcpp::CallbackGroup::SharedPtr co_cli_write_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;
  rclcpp::CallbackGroup::SharedPtr status_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_conveyor_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_stopper_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr normal_timer_cbg_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heater_timer_;
  rclcpp::TimerBase::SharedPtr con_state_timer_;
  rclcpp::TimerBase::SharedPtr remain_length_timer_;
  rclcpp::TimerBase::SharedPtr once_timer_;

  rclcpp::Publisher<PackagingMachineStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_publisher_;
  rclcpp::Publisher<PackagingMachineInfo>::SharedPtr info_publisher_;
  rclcpp::Publisher<UnbindRequest>::SharedPtr unbind_mtrl_box_publisher_;
  rclcpp::Publisher<Bool>::SharedPtr skip_pkg_publisher_;

  rclcpp::Publisher<COData>::SharedPtr tpdo_pub_;
  rclcpp::Subscription<COData>::SharedPtr rpdo_sub_;

  rclcpp::Service<Trigger>::SharedPtr init_pkg_mac_service_;
  rclcpp::Service<SetBool>::SharedPtr heater_service_;
  rclcpp::Service<SetBool>::SharedPtr cutter_service_;
  rclcpp::Service<SetBool>::SharedPtr stopper_service_;
  rclcpp::Service<SetBool>::SharedPtr mtrl_box_gate_service_;
  rclcpp::Service<SetBool>::SharedPtr conveyor_service_;
  rclcpp::Service<SetBool>::SharedPtr pill_gate_service_;
  rclcpp::Service<SetBool>::SharedPtr roller_service_;
  rclcpp::Service<Trigger>::SharedPtr squeezer_service_;
  rclcpp::Service<UInt8Srv>::SharedPtr pkg_len_service_;
  rclcpp::Service<Trigger>::SharedPtr print_one_pkg_service_;
  rclcpp::Service<Trigger>::SharedPtr print_one_pkg_wo_squ_service_;
  rclcpp::Service<SetBool>::SharedPtr state_ctrl_service_;
  rclcpp::Service<SetBool>::SharedPtr skip_pkg_service_;
  rclcpp::Service<SetBool>::SharedPtr enable_heater_service_;
  rclcpp::Service<UInt32Srv>::SharedPtr update_package_service_;
  rclcpp::Service<UInt32Srv>::SharedPtr update_thermal_service_;

  rclcpp::Client<CORead>::SharedPtr co_read_client_;
  rclcpp::Client<COWrite>::SharedPtr co_write_client_;

  rclcpp_action::Server<PackagingOrder>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const PackagingOrder::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);
  void handle_accepted(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);

  void order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);
  void skip_order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);
  
  void init_handle(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);
  void heater_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void cutter_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void stopper_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void mtrl_box_gate_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void conveyor_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void pill_gate_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void roller_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void squeezer_handle(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);
  void pkg_len_handle(
    const std::shared_ptr<UInt8Srv::Request> request, 
    std::shared_ptr<UInt8Srv::Response> response);
  void print_one_pkg_handle(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);
  void print_one_pkg_wo_squ_handle(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);
  void state_ctrl_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void skip_pkg_ctrl_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void enable_heater_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  void update_package_handle(
    const std::shared_ptr<UInt32Srv::Request> request, 
    std::shared_ptr<UInt32Srv::Response> response);
  void update_thermal_handle(
    const std::shared_ptr<UInt32Srv::Request> request, 
    std::shared_ptr<UInt32Srv::Response> response);

  void rpdo_cb(const COData::SharedPtr msg);

}; // class PackagingMachineNode

#endif  // PACKAGING_MACHINE_NODE_HPP_