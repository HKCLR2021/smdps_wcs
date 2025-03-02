#include "wcs/dis_station_node.hpp"

void DispenserStationNode::initiate(void)
{
  if (!write_opcua_value(initiate_id, true)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }

  // wait forever until triggered
  if (rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Waiting the initiate signal...");
    std::unique_lock<std::mutex> lock(init_signal.cv_mutex_);
    init_signal.cv_.wait(lock, [this]() { 
      return init_signal.is_triggered_; 
    });
    init_signal.is_triggered_ = false;
  }

  if (!write_opcua_value(initiate_id, false)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Initiated the Dispenser Station [%d] successfully", status_->id);
}

void DispenserStationNode::clear_cmd_req(void)
{
  if (!write_opcua_value(cmd_req_id, false))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }
}

void DispenserStationNode::reset(void)
{
  if (!write_opcua_value(reset_id, true))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }

  // wait forever until triggered
  if (rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Waiting the reset signal...");
    std::unique_lock<std::mutex> lock(reset_signal.cv_mutex_);
    reset_signal.cv_.wait(lock, [this]() { 
      return reset_signal.is_triggered_; 
    });
    reset_signal.is_triggered_ = false;
  }

  if (!write_opcua_value(reset_id, false))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }
}

void DispenserStationNode::test_bin(void)
{
  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    if (!status_->unit_status[i].enable)
      continue;

    const uint8_t id = i + 1;
    if (!write_opcua_value(bin_open_req_id[id], true))
    {
      RCLCPP_ERROR(this->get_logger(), "unit_id: %d, Error occur in %s", id, __FUNCTION__);
    }
    
    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(this->get_logger(), "The bins are opened in Dispenser Station [%d]", status_->id);
  std::this_thread::sleep_for(1s);

  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    if (!status_->unit_status[i].enable)
      continue;

    const uint8_t id = i + 1;
    if (!write_opcua_value(bin_close_req_id[id], true))
    {
      RCLCPP_ERROR(this->get_logger(), "unit_id: %d, Error occur in %s", id, __FUNCTION__);
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(this->get_logger(), "The bins are closed in Dispenser Station [%d]", status_->id);
  RCLCPP_INFO(this->get_logger(), "Initiated the Open and Close state");
}

void DispenserStationNode::test_baffle(void)
{
  std::vector<std::future<opcua::StatusCode>> futures;
  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    if (!status_->unit_status[i].enable)
      continue;

    const uint8_t id = i + 1;
    if (!write_opcua_value(baffle_open_req_id[id], true))
    {
      RCLCPP_ERROR(this->get_logger(), "unit_id: %d, Error occur in %s", id, __FUNCTION__);
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(this->get_logger(), "The baffles are opened in Dispenser Station [%d]", status_->id);
  std::this_thread::sleep_for(1s);

  for (size_t i = 0; i < NO_OF_UNITS; i++)
  {
    if (!status_->unit_status[i].enable)
      continue;

    const uint8_t id = i + 1;
    if (!write_opcua_value(baffle_close_req_id[id], true))
    {
      RCLCPP_ERROR(this->get_logger(), "unit_id: %d, Error occur in %s", id, __FUNCTION__);
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(this->get_logger(), "The baffles are closed in Dispenser Station [%d]", status_->id);
  RCLCPP_INFO(this->get_logger(), "Initiated the Open and Close state");
}

void DispenserStationNode::clear_req(const opcua::NodeId req_node_id, const opcua::NodeId state_node_id)
{
  std::chrono::milliseconds freq = 500ms;
  rclcpp::Rate loop_rate(freq); 

  while(rclcpp::ok() && cli.isConnected())
  {
    std::shared_ptr<bool> val = std::make_shared<bool>();
    if (!read_opcua_value(state_node_id, val))
      continue;

    if (*val)
    {
      RCLCPP_INFO(this->get_logger(), "The state changed to true");
      break;
    }

    loop_rate.sleep();
  }

  if (!write_opcua_value(req_node_id, false))
  {
    RCLCPP_ERROR(this->get_logger(), "Error occur in %s", __FUNCTION__);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Clear the Request of Dispenser Station [%d] successfully", status_->id);
}