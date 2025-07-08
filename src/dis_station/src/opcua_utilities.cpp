#include "dis_station/dis_station_node.hpp"

template bool DispenserStationNode::read_opcua_value<bool>(const opcua::NodeId&, std::shared_ptr<bool>);
template bool DispenserStationNode::read_opcua_value<int16_t>(const opcua::NodeId&, std::shared_ptr<int16_t>);
template bool DispenserStationNode::write_opcua_value<bool>(const opcua::NodeId&, bool);
template bool DispenserStationNode::write_opcua_value<int16_t>(const opcua::NodeId&, int16_t);

template <typename T>
bool DispenserStationNode::write_opcua_value(const opcua::NodeId& node_id, T value) 
{
  // constexpr int max_attempts = 100;
  // int attempts = 0;
  bool done = false;

  while (rclcpp::ok() && !done)
  {
    try
    {
      if (!cli.isConnected())
      {
        RCLCPP_ERROR(this->get_logger(), "OPC UA Server is not connected in %s", __FUNCTION__);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      opcua::Variant var;
      var = value;
      std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, node_id, var, opcua::useFuture);

      // Wait with timeout and shutdown awareness
      if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) 
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout in %s", __FUNCTION__);
        continue;
      }

      opcua::StatusCode code = future.get();
      if (code != UA_STATUSCODE_GOOD) 
      {
        RCLCPP_ERROR(this->get_logger(), "The statusCode %s in %s", std::to_string(code).c_str(), __FUNCTION__);
        continue;
      }

      done = true;
    }
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in %s: %s", __FUNCTION__, e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    catch (...) 
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in %s", __FUNCTION__);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  return true;
}

template <typename T>
bool DispenserStationNode::read_opcua_value(const opcua::NodeId& node_id, std::shared_ptr<T> ptr_value) 
{
  // constexpr int max_attempts = 100;
  // int attempts = 0;
  bool done = false;

  while (rclcpp::ok() && !done)
  {
    try
    {
      if (!cli.isConnected())
      {
        RCLCPP_ERROR(this->get_logger(), "OPC UA Server is not connected in %s", __FUNCTION__);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      std::future<opcua::Result<opcua::Variant>> future = opcua::services::readValueAsync(cli, node_id, opcua::useFuture);

      // Wait with timeout and shutdown awareness
      if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) 
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout in %s", __FUNCTION__);
        continue;
      }

      opcua::Result<opcua::Variant> result = future.get();
      if (result.code() != UA_STATUSCODE_GOOD) 
      {
        RCLCPP_ERROR(this->get_logger(), "The result code: %s", std::to_string(result.code()).c_str());
        continue;
      }

      std::optional<T> val = result.value().scalar<T>();
      if (!val) 
      {
        RCLCPP_ERROR(this->get_logger(), "Value type mismatch or extraction failed in %s", __FUNCTION__);
        continue;
      }

      *ptr_value = val.value();
      done = true;
    }
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in %s: %s", __FUNCTION__, e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    catch (...) 
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in %s", __FUNCTION__);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  return true;
}

// template <typename T>
// bool DispenserStationNode::write_opcua_value(const opcua::NodeId& node_id, T value) 
// {
//   if (!cli.isConnected())
//     return false;

//   opcua::Variant var;
//   var = value;
//   std::future<opcua::StatusCode> future = opcua::services::writeValueAsync(cli, node_id, var, opcua::useFuture);

//   // Wait with timeout and shutdown awareness
//   if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) 
//   {
//     RCLCPP_ERROR(this->get_logger(), "Timeout in %s", __FUNCTION__);
//     return false;
//   }

//   opcua::StatusCode code = future.get();
//   if (code != UA_STATUSCODE_GOOD) 
//   {
//     RCLCPP_ERROR(this->get_logger(), "The statusCode %s in %s", std::to_string(code).c_str(), __FUNCTION__);
//     return false;
//   }

//   return true;
// }

// template <typename T>
// bool DispenserStationNode::read_opcua_value(const opcua::NodeId& node_id, std::shared_ptr<T> value) 
// {
//   if (!cli.isConnected())
//     return false;

//   std::future<opcua::Result<opcua::Variant>> future = opcua::services::readValueAsync(cli, node_id, opcua::useFuture);

//   // Wait with timeout and shutdown awareness
//   if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) 
//   {
//     RCLCPP_ERROR(this->get_logger(), "Timeout in %s", __FUNCTION__);
//     return false;
//   }

//   opcua::Result<opcua::Variant> result = future.get();
//   if (result.code() != UA_STATUSCODE_GOOD) 
//   {
//     RCLCPP_ERROR(this->get_logger(), "The result code: %s", std::to_string(result.code()).c_str());
//     return false;
//   }

//   std::optional<T> val = result.value().scalar<T>();
//   if (!val) 
//   {
//     RCLCPP_ERROR(this->get_logger(), "Value type mismatch or extraction failed in %s", __FUNCTION__);
//     return false;
//   }

//   *value = val.value();
//   return true;
// }

bool DispenserStationNode::wait_for_futures(std::vector<std::future<opcua::StatusCode>> &futures) 
{
  bool all_good = true;

  while (!futures.empty() && rclcpp::ok()) 
  {
    for (auto it = futures.begin(); it != futures.end();) 
    {
      auto status = it->wait_for(100ms);
      if (status == std::future_status::ready) 
      {
        if (it->get() != UA_STATUSCODE_GOOD) 
          all_good = false;
        
        it = futures.erase(it);
      } 
      else 
      {
        ++it;
      }
    }
  }

  return all_good && rclcpp::ok();
}

void DispenserStationNode::wait_for_opcua_connection(const std::chrono::milliseconds freq, const std::chrono::seconds timeout)
{
  RCLCPP_DEBUG(this->get_logger(), "Started to wait the OPCUA connection <<<<<<<<<<<<<");

  // rclcpp::Rate loop_rate(freq); 
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (cli.isConnected())
      {
        RCLCPP_INFO(this->get_logger(), "OPC UA connection established.");
        break;
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for OPC UA connection after %ld seconds.", timeout.count());
      throw std::runtime_error("OPC UA connection timeout");
    }
    
    RCLCPP_ERROR(this->get_logger(), "Waiting for opcua connection (%ld ms to retry)...", freq.count());
    std::this_thread::sleep_for(std::chrono::milliseconds(freq));
    // loop_rate.sleep();
  }
}

const std::string DispenserStationNode::form_opcua_url(void)
{
  if (!ip_.empty() && !port_.empty())
    return "opc.tcp://" + ip_ + ":" + port_;
  else
    throw std::runtime_error(std::string("%s failed", __FUNCTION__));
}

constexpr std::string_view DispenserStationNode::get_enum_name(opcua::NodeClass node_class) 
{
  switch (node_class) 
  {
  case opcua::NodeClass::Object:
    return "Object";
  case opcua::NodeClass::Variable:
    return "Variable";
  case opcua::NodeClass::Method:
    return "Method";
  case opcua::NodeClass::ObjectType:
    return "ObjectType";
  case opcua::NodeClass::VariableType:
    return "VariableType";
  case opcua::NodeClass::ReferenceType:
    return "ReferenceType";
  case opcua::NodeClass::DataType:
    return "DataType";
  case opcua::NodeClass::View:
    return "View";
  default:
    return "Unknown";
  }
}

constexpr std::string_view DispenserStationNode::get_log_level_name(opcua::LogLevel level) 
{
  switch (level) 
  {
  case opcua::LogLevel::Trace:
    return "trace";
  case opcua::LogLevel::Debug:
    return "debug";
  case opcua::LogLevel::Info:
    return "info";
  case opcua::LogLevel::Warning:
    return "warning";
  case opcua::LogLevel::Error:
    return "error";
  case opcua::LogLevel::Fatal:
    return "fatal";
  default:
    return "unknown";
  }
}
 
constexpr std::string_view DispenserStationNode::get_log_category_name(opcua::LogCategory category) 
{
  switch (category) 
  {
  case opcua::LogCategory::Network:
    return "network";
  case opcua::LogCategory::SecureChannel:
    return "channel";
  case opcua::LogCategory::Session:
    return "session";
  case opcua::LogCategory::Server:
    return "server";
  case opcua::LogCategory::Client:
    return "client";
  case opcua::LogCategory::Userland:
    return "userland";
  case opcua::LogCategory::SecurityPolicy:
    return "securitypolicy";
  default:
    return "unknown";
  }
}

void DispenserStationNode::logger_wrapper(opcua::LogLevel level, opcua::LogCategory category, std::string_view msg)
{
  switch (level) 
  {
  case opcua::LogLevel::Debug:
    RCLCPP_DEBUG(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
  case opcua::LogLevel::Trace:
  case opcua::LogLevel::Info:
    RCLCPP_INFO(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
    break;
  case opcua::LogLevel::Warning:
    RCLCPP_WARN(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
    break;
  case opcua::LogLevel::Error:
    RCLCPP_ERROR(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
    break;
  case opcua::LogLevel::Fatal:
    RCLCPP_FATAL(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "[%s] %s", std::string(get_log_category_name(category)).c_str(), std::string(msg).c_str());
    break;
  }
}