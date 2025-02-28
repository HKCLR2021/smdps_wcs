#include "wcs/dis_station_node.hpp"

void DispenserStationNode::heartbeat_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;

  RCLCPP_DEBUG(this->get_logger(), ">>>> Heartbeat data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  const std::lock_guard<std::mutex> lock(mutex_);
  status_->heartbeat = true;
  heartbeat_counter_ = 0;
  RCLCPP_DEBUG(this->get_logger(), ">>>> Reset heartbeat_counter_");
}

void DispenserStationNode::general_bool_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value, const std::string name, std::shared_ptr<bool> ptr)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;

  if (!name.empty())
  {
    RCLCPP_INFO(this->get_logger(), ">>>> %s data change notification, value: %s", name.c_str(), *val ? "true" : "false");
    RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
    RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  *ptr = *val;
}

void DispenserStationNode::alm_code_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<int16_t> val = value.value().scalar<int16_t>();
  if (!val)
    return;
  
  if (val.value() == 0)
    RCLCPP_DEBUG(this->get_logger(), ">>>> ALM Code data change notification, value: %d", *val);
  else
    RCLCPP_ERROR(this->get_logger(), ">>>> ALM Code data change notification, value: %d", *val);
  
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  const std::lock_guard<std::mutex> lock(mutex_);
  status_->error_code = val.value();
}

void DispenserStationNode::completed_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> Completed data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  if (val.value())
  {
    RCLCPP_INFO(this->get_logger(), "A dispense request is completed in Dispenser Station [%d]", status_->id);
    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(com_signal.cv_mutex_);
      com_signal.is_triggered_ = true;
    }
    com_signal.cv_.notify_one();  // Wake up the waiting dis_req_handle

    RCLCPP_INFO(this->get_logger(), "Notified the dis_req_handle to continue");
  }
}

void DispenserStationNode::dispensing_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> Dispensing data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);
}

void DispenserStationNode::initiate_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> Initiate data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  if (val.value())
  {
    RCLCPP_INFO(this->get_logger(), "Trigger the Initiate in Dispenser Station [%d]", status_->id);
    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(init_signal.cv_mutex_);
      init_signal.is_triggered_ = true;
    }
    init_signal.cv_.notify_one();  // Wake up the initiate action

    RCLCPP_INFO(this->get_logger(), "Notified the Initiate to continue");
  }
}

void DispenserStationNode::reset_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> Reset data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  if (val.value())
  {
    RCLCPP_INFO(this->get_logger(), "Trigger the Reset in Dispenser Station [%d]", status_->id);
    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(reset_signal.cv_mutex_);
      reset_signal.is_triggered_ = true;
    }
    reset_signal.cv_.notify_one();  // Wake up the reset action

    RCLCPP_INFO(this->get_logger(), "Notified the Reset to continue");
  }
}

void DispenserStationNode::cmd_amt_cb(uint32_t sub_id, uint32_t mon_id, const opcua::DataValue &value)
{
  std::optional<int16_t> val = value.value().scalar<int16_t>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> CmdAmount data change notification, value: %s", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  if (val.value() > 0)
  {
    RCLCPP_INFO(this->get_logger(), "Trigger the CmdAmount in Dispenser Station [%d]", status_->id);
    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(cmd_amt_signal.cv_mutex_);
      cmd_amt_signal.is_triggered_ = true;
      cmd_amt_signal.val = val.value();
    }
    cmd_amt_signal.cv_.notify_one();  // Wake up the CmdAmount action

    RCLCPP_INFO(this->get_logger(), "Notified the CmdAmount to continue");
  }
}


void DispenserStationNode::open_close_req_cb(
  uint32_t sub_id, 
  uint32_t mon_id, 
  const opcua::DataValue &value, 
  const opcua::NodeId req_node_id, 
  const opcua::NodeId state_node_id)
{
  std::optional<bool> val = value.value().scalar<bool>();
  if (!val)
    return;
  
  RCLCPP_INFO(this->get_logger(), ">>>> %s data change notification, value: %s", "Open/Close", *val ? "true" : "false");
  RCLCPP_DEBUG(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %d", mon_id);

  if (val.value())
  {
    RCLCPP_INFO(this->get_logger(), "Try to clear the open/close request Dispenser Station [%d]", status_->id);
    std::thread(std::bind(&DispenserStationNode::clear_req, this, req_node_id, state_node_id)).detach();
  }
}
