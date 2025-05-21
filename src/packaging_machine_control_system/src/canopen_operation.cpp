#include "packaging_machine_control_system/packaging_machine_node.hpp"

bool PackagingMachineNode::call_co_write(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  using ServiceResponseFuture = rclcpp::Client<COWrite>::SharedFuture;
  auto response_received_cb = [this](ServiceResponseFuture future) {
    auto srv_response = future.get();

    if (srv_response) 
      RCLCPP_DEBUG(this->get_logger(), "Inside the COWrite Callback OK");
    else 
      RCLCPP_ERROR(this->get_logger(), "Inside the COWrite Callback NOT OK");

    // if (rclcpp::ok())
    // {
    //   std::lock_guard<std::mutex> lock(co_write_signal.cv_mutex_);
    //   co_write_signal.is_triggered_ = true;
    // }

    // co_write_signal.cv_.notify_one();  // Wake up service
  };

  auto future = co_write_client_->async_send_request(request, response_received_cb);
  // future.wait();

  // if (rclcpp::ok())
  // {
  //   RCLCPP_INFO(this->get_logger(), "Waiting the canopen write signal...");
  //   std::unique_lock<std::mutex> lock(co_write_signal.cv_mutex_);

  //   co_write_signal.cv_.wait(lock, [this]() { 
  //     return co_write_signal.is_triggered_; 
  //   });

  //   co_write_signal.is_triggered_ = false;
  // }

  // return true;
  std::future_status status = future.wait_for(200ms);
  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(this->get_logger(), "%s wait_for OK", __FUNCTION__);
    return true; 
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "%s wait_for timeout", __FUNCTION__);
    return false;
  default: 
    RCLCPP_ERROR(this->get_logger(), "%s wait_for NOT OK", __FUNCTION__);
    return false;
  }
}

bool PackagingMachineNode::call_co_read(uint16_t index, uint8_t subindex, std::shared_ptr<uint32_t> data)
{
  std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

  request->index = index;
  request->subindex = subindex;

  co_read_wait_for_service();

  using ServiceResponseFuture = rclcpp::Client<CORead>::SharedFuture;
  auto response_received_cb = [this, data](ServiceResponseFuture future) {
    auto srv_response = future.get();
    
    if (srv_response) 
    {
      *data = srv_response->data;
      RCLCPP_DEBUG(this->get_logger(), "Inside the CORead Callback OK, data: %d", *data);
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the CORead Callback NOT OK");
    }

    // if (rclcpp::ok())
    // {
    //   std::lock_guard<std::mutex> lock(co_read_signal.cv_mutex_);
    //   co_read_signal.is_triggered_ = true;
    // }

    // co_read_signal.cv_.notify_one();  // Wake up service
  };
 
  auto future = co_read_client_->async_send_request(request, response_received_cb);
  // future.wait();

  // if (rclcpp::ok())
  // {
  //   RCLCPP_INFO(this->get_logger(), "Waiting the canopen read signal...");
  //   std::unique_lock<std::mutex> lock(co_read_signal.cv_mutex_);

  //   co_read_signal.cv_.wait(lock, [this]() { 
  //     return co_read_signal.is_triggered_; 
  //   });

  //   co_read_signal.is_triggered_ = false;
  // }

  // return true;
  std::future_status status = future.wait_for(200ms);
  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(this->get_logger(), "%s wait_for OK", __FUNCTION__);
    return true;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "%s wait_for timeout", __FUNCTION__);
    return false;
  default: 
    RCLCPP_ERROR(this->get_logger(), "%s wait_for NOT OK", __FUNCTION__);
    return false;
  }
}

// Don't use this function
bool PackagingMachineNode::call_co_write_w_spin(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  auto future = co_write_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
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
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
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
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }

    RCLCPP_ERROR(this->get_logger(), "COWrite Service not available, waiting again...");
    // std::this_thread::sleep_for(1s);
  }
}

void PackagingMachineNode::co_read_wait_for_service(void)
{
  while (!co_read_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }

    RCLCPP_ERROR(this->get_logger(), "CORead Service not available, waiting again...");
    // std::this_thread::sleep_for(1s);
  }
}