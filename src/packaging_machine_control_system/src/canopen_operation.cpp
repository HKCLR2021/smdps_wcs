#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::call_co_write(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  // using ServiceResponseFuture = rclcpp::Client<COWrite>::SharedFuture;
  // auto response_received_cb = [this](ServiceResponseFuture future) {
  //   auto srv_response = future.get();

  //   if (srv_response) 
  //     RCLCPP_DEBUG(get_logger(), "Inside the COWrite Callback OK");
  //   else 
  //     RCLCPP_ERROR(get_logger(), "Inside the COWrite Callback NOT OK");
  // };

  // auto future = co_write_client_->async_send_request(request, response_received_cb);
  auto future = co_write_client_->async_send_request(request);
  std::future_status status = future.wait_for(1s);

  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(get_logger(), "%s wait_for OK", __FUNCTION__);
    return true; 
  case std::future_status::timeout:
    RCLCPP_ERROR(get_logger(), "%s wait_for timeout", __FUNCTION__);
    return false;
  default: 
    RCLCPP_ERROR(get_logger(), "%s wait_for NOT OK", __FUNCTION__);
    return false;
  }

  // auto response = future.get();

  // if (!response->success)
  // {
  //   RCLCPP_INFO(get_logger(), "Service %s call failed", __FUNCTION__);
  //   return false;
  // }

  return false;
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
    RCLCPP_DEBUG(get_logger(), "%s wait_for OK", __FUNCTION__);
    break;
  case std::future_status::timeout:
    RCLCPP_ERROR(get_logger(), "%s wait_for timeout", __FUNCTION__);
    return std::nullopt;
  default: 
    RCLCPP_ERROR(get_logger(), "%s wait_for NOT OK", __FUNCTION__);
    return std::nullopt;
  }

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Service %s call failed", __FUNCTION__);
    return std::nullopt;
  }
  
  RCLCPP_DEBUG(get_logger(), "Inside the CORead, response->data: %d", response->data);
  return std::make_optional(std::move(response->data));
}

// bool PackagingMachineNode::call_co_read(uint16_t index, uint8_t subindex, std::shared_ptr<uint32_t> data)
// {
//   std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

//   request->index = index;
//   request->subindex = subindex;

//   co_read_wait_for_service();

//   // using ServiceResponseFuture = rclcpp::Client<CORead>::SharedFuture;
//   // auto response_received_cb = [this, data](ServiceResponseFuture future) {
//   //   auto srv_response = future.get();
    
//   //   if (srv_response) 
//   //   {
//   //     *data = srv_response->data;
//   //     RCLCPP_WARN(get_logger(), "Inside the CORead Callback OK, data: %d", *data);
//   //   } 
//   //   else 
//   //   {
//   //     RCLCPP_ERROR(get_logger(), "Inside the CORead Callback NOT OK");
//   //   }
//   // };
 
//   // auto future = co_read_client_->async_send_request(request, response_received_cb);
//   auto future = co_read_client_->async_send_request(request);
//   std::future_status status = future.wait_for(1s);

//   switch (status)
//   {
//   case std::future_status::ready:
//     RCLCPP_DEBUG(get_logger(), "%s wait_for OK", __FUNCTION__);
//     return true;
//   case std::future_status::timeout:
//     RCLCPP_ERROR(get_logger(), "%s wait_for timeout", __FUNCTION__);
//     return false;
//   default: 
//     RCLCPP_ERROR(get_logger(), "%s wait_for NOT OK", __FUNCTION__);
//     return false;
//   }

//   auto response = future.get();

//   if (!response->success)
//   {
//     RCLCPP_INFO(get_logger(), "Service %s call failed", __FUNCTION__);
//     return false;
//   }

//   std::shared_ptr val = std::make_shared<uint32_t>(std::move(response->data));
//   data = std::move(val);
  
//   RCLCPP_WARN(get_logger(), "Inside the CORead Callback OK, response->data: %d", response->data);
//   RCLCPP_WARN(get_logger(), "data: %d", *data);

//   RCLCPP_DEBUG(get_logger(), "Inside the CORead Callback OK");
// }

// Don't use this function
bool PackagingMachineNode::call_co_write_w_spin(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  auto future = co_write_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    return response->success;
  } 
  else 
  {
    return false;
  }
}

// Don't use this function
bool PackagingMachineNode::call_co_read_w_spin(uint16_t index, uint8_t subindex, std::shared_ptr<uint32_t> data)
{
  std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

  request->index = index;
  request->subindex = subindex;

  co_read_wait_for_service();
 
  auto future = co_read_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    *data = response->data;
    return response->success;
  } 
  else 
  {
    return false;
  }
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
    // std::this_thread::sleep_for(1s);
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
    // std::this_thread::sleep_for(1s);
  }
}