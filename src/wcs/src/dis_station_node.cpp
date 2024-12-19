#include "wcs/dis_station_node.hpp"

DispenserStationNode::DispenserStationNode(const rclcpp::NodeOptions& options)
: Node("dis_station_node", options),
  status_(std::make_shared<DispenserStationStatus>()),
  cli_started_(std::atomic<bool>{false})
{
  this->declare_parameter<uint8_t>("id", 0);
  this->declare_parameter<std::string>("ip", "");
  this->declare_parameter<std::string>("port", "");
  this->declare_parameter<bool>("simulation", false);

  this->get_parameter("id", status_->id);
  this->get_parameter("ip", ip_);
  this->get_parameter("port", port_);
  this->get_parameter("simulation", sim_);

  status_pub_ = this->create_publisher<DispenserStationStatus>("dispenser_station_status", 10);
  status_timer_ = this->create_wall_timer(1s, std::bind(&DispenserStationNode::dis_station_status_cb, this));

  dis_req_srv_ = this->create_service<DispenseDrug>(
    "dispense_request", 
    std::bind(&DispenserStationNode::dis_req_handle, 
      this, 
      _1, 
      _2));

  if (!init_opcua_cli())
  {
    RCLCPP_ERROR(this->get_logger(), "init opcua client error occurred");
    rclcpp::shutdown();
  }

  cli_thread_ = std::thread(std::bind(&DispenserStationNode::start_opcua_cli, this)); 
  RCLCPP_INFO(this->get_logger(), "OPCUA Server: %s", form_opcua_url().c_str());

  RCLCPP_INFO(this->get_logger(), "dispenser station node is up");
}

DispenserStationNode::~DispenserStationNode()
{
  if (cli_started_.load()) 
  {
    cli_started_.store(false);
    cli.stop();
  }
  if (cli_thread_.joinable()) 
  {
    cli_thread_.join();
  }
}

void DispenserStationNode::dis_station_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_pub_->publish(*status_);
}

void DispenserStationNode::dis_req_handle(const std::shared_ptr<DispenseDrug::Request> req, std::shared_ptr<DispenseDrug::Response> res)
{

}

bool DispenserStationNode::init_opcua_cli(void)
{
  auto logger = [this](auto level, auto category, auto msg) 
  {
    switch (level) 
    {
    case opcua::LogLevel::Trace:
    case opcua::LogLevel::Debug:
    case opcua::LogLevel::Info:
    case opcua::LogLevel::Warning:
      RCLCPP_INFO(this->get_logger(), "[%s] %s", std::string(getLogCategoryName(category)).c_str(), std::string(msg).c_str());
      break;
    case opcua::LogLevel::Error:
    case opcua::LogLevel::Fatal:
    default:
      RCLCPP_ERROR(this->get_logger(), "[%s] %s", std::string(getLogCategoryName(category)).c_str(), std::string(msg).c_str());
      break;
    }
  };

  opcua::ClientConfig config;
  cli.config().setLogger(logger);
  cli.onDisconnected(std::bind(&DispenserStationNode::disconnected_cb, this));
  cli.onConnected(std::bind(&DispenserStationNode::connected_cb, this));

  return true;
}

void DispenserStationNode::start_opcua_cli(void)
{
  cli_started_.store(true);

  while (cli_started_.load()) 
  {
    try 
    {
      if (!cli.isConnected())
      {
        cli.connect(form_opcua_url());
      }
      cli.run();
    } 
    catch (const opcua::BadStatus& e) 
    {
      cli.disconnect();
      RCLCPP_ERROR(this->get_logger(), "Error: %s, Retry to connect in 1 seconds", e.what());
      std::this_thread::sleep_for(1s);
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "caught an unknown exception!!!");
      cli_started_.store(false);
      rclcpp::shutdown();
    }
  }
}

inline const std::string DispenserStationNode::form_opcua_url(void)
{
  if (!ip_.empty() && !port_.empty())
    return "opc.tcp://" + ip_ + ":" + port_;
  else
    throw std::runtime_error(std::string("%s failed", __FUNCTION__));
}

void DispenserStationNode::disconnected_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  // is_connected_ = false;
  RCLCPP_INFO(this->get_logger(), ">>> disconnected to opcua server: %s:%s", ip_.c_str(), port_.c_str());
}

void DispenserStationNode::connected_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  // is_connected_ = true;
  RCLCPP_INFO(this->get_logger(), ">>> connected to opcua server: %s:%s", ip_.c_str(), port_.c_str());
}

constexpr std::string_view DispenserStationNode::getLogLevelName(opcua::LogLevel level) 
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
 
constexpr std::string_view DispenserStationNode::getLogCategoryName(opcua::LogCategory category) 
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<DispenserStationNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}