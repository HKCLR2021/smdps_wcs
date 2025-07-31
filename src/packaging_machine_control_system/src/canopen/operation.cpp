#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::call_co_write(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  auto future = co_write_client_->async_send_request(request);
  std::future_status status = future.wait_for(1s);

  switch (status)
  {
  case std::future_status::ready:
    break; 
  case std::future_status::timeout:
    RCLCPP_ERROR(get_logger(), "%s wait_for timeout", __FUNCTION__);
    return false;
  case std::future_status::deferred: 
    RCLCPP_ERROR(get_logger(), "%s wait_for deferred", __FUNCTION__);
    return false;
  }

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "Service %s call failed", __FUNCTION__);
    return false;
  }

  RCLCPP_DEBUG(get_logger(), "%s CANopen SDO write successfully", __FUNCTION__);
  return true;
}

std::optional<uint32_t> PackagingMachineNode::call_co_read(uint16_t index, uint8_t subindex)
{
  std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

  request->index = index;
  request->subindex = subindex;

  co_read_wait_for_service();

  auto future = co_read_client_->async_send_request(request);
  std::future_status status = future.wait_for(1s);

  switch (status)
  {
  case std::future_status::ready:
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(get_logger(), "%s wait_for timeout", __FUNCTION__);
    return std::nullopt;
  case std::future_status::deferred: 
    RCLCPP_ERROR(get_logger(), "%s wait_for deferred", __FUNCTION__);
    return std::nullopt;
  }

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Service %s call failed", __FUNCTION__);
    return std::nullopt;
  }
  
  RCLCPP_DEBUG(get_logger(), "%s CANopen SDO write successfully [data: %d]", __FUNCTION__, response->data);
  return std::make_optional(std::move(response->data));
}

void PackagingMachineNode::co_write_wait_for_service(void)
{
  while (!co_write_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }

    RCLCPP_ERROR(get_logger(), "COWrite Service not available, waiting again...");
  }
}

void PackagingMachineNode::co_read_wait_for_service(void)
{
  while (!co_read_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }

    RCLCPP_ERROR(get_logger(), "CORead Service not available, waiting again...");
  }
}