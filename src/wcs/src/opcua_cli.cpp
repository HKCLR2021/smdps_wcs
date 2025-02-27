#include "wcs/dis_station_node.hpp"

bool DispenserStationNode::init_opcua_cli(void)
{
  opcua::ClientConfig config;
  cli.config().setTimeout(OPCUA_TIMEOUT);
  cli.config().setLogger(std::bind(&DispenserStationNode::logger_wrapper, this, _1, _2, _3));

  cli.onDisconnected(std::bind(&DispenserStationNode::disconnected_cb, this));
  cli.onConnected(std::bind(&DispenserStationNode::connected_cb, this));
  cli.onSessionActivated(std::bind(&DispenserStationNode::session_activated_cb, this));
  cli.onSessionClosed(std::bind(&DispenserStationNode::session_closed_cb, this));
  cli.onInactive(std::bind(&DispenserStationNode::inactive_cb, this));

  return true;
}

void DispenserStationNode::start_opcua_cli(void)
{
  cli_started_.store(true);
  std::chrono::seconds sleep_time = 1s;

  while (cli_started_.load() && rclcpp::ok()) 
  {
    try 
    {
      cli.connectAsync(form_opcua_url());
      cli.run();
    }
    catch (const opcua::BadStatus& e) 
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cli.disconnectAsync();
      RCLCPP_ERROR(this->get_logger(), "Error: %s, Retry to connect Station [%d] in %ld seconds", e.what(), status_->id, sleep_time.count());
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Caught an unknown exception!!!");
      cli_started_.store(false);
      rclcpp::shutdown();
    }

    std::this_thread::sleep_for(sleep_time);
  }
}

void DispenserStationNode::create_sub_async(void)
{
  opcua::SubscriptionParameters sub_params{};
  sub_params.publishingInterval = 500;
  sub_params.lifetimeCount = 2400;
  sub_params.maxKeepAliveCount = 10;
  sub_params.maxNotificationsPerPublish = 0;
  sub_params.priority = 0;

  opcua::services::createSubscriptionAsync(
    cli,
    sub_params,
    true,
    std::bind(&DispenserStationNode::sub_status_change_cb, this, _1, _2),
    std::bind(&DispenserStationNode::sub_deleted_cb, this, _1),
    [&] (opcua::CreateSubscriptionResponse& res) {
      RCLCPP_INFO(this->get_logger(), ">>>> Subscription created:");
      RCLCPP_INFO(this->get_logger(), ">>>> - status code: %s", std::to_string(res.responseHeader().serviceResult()).c_str());
      RCLCPP_INFO(this->get_logger(), ">>>> - subscription id: %s", std::to_string(res.subscriptionId()).c_str());

      opcua::MonitoringParametersEx monitoring_params{};
      monitoring_params.samplingInterval = 250.0;
      monitoring_params.queueSize = 1;
      monitoring_params.discardOldest = true;

      opcua::services::createMonitoredItemDataChangeAsync(
        cli,
        res.subscriptionId(),
        opcua::ReadValueId(heartbeat_id, opcua::AttributeId::Value),
        opcua::MonitoringMode::Reporting,
        monitoring_params,
        std::bind(&DispenserStationNode::heartbeat_cb, this, _1, _2, _3),
        std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "Heartbeat"), 
        std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "Heartbeat")
      );

      opcua::services::createMonitoredItemDataChangeAsync(
        cli,
        res.subscriptionId(),
        opcua::ReadValueId(alm_code_id, opcua::AttributeId::Value),
        opcua::MonitoringMode::Reporting,
        monitoring_params,
        std::bind(&DispenserStationNode::alm_code_cb, this, _1, _2, _3),
        std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "ALM Code"), 
        std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "ALM Code")
      );
            
      opcua::services::createMonitoredItemDataChangeAsync(
        cli,
        res.subscriptionId(),
        opcua::ReadValueId(completed_id, opcua::AttributeId::Value),
        opcua::MonitoringMode::Reporting,
        monitoring_params,
        std::bind(&DispenserStationNode::completed_cb, this, _1, _2, _3),
        std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "Completed"), 
        std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "Completed")
      );

      opcua::services::createMonitoredItemDataChangeAsync(
        cli,
        res.subscriptionId(),
        opcua::ReadValueId(dispensing_id, opcua::AttributeId::Value),
        opcua::MonitoringMode::Reporting,
        monitoring_params,
        std::bind(&DispenserStationNode::dispensing_cb, this, _1, _2, _3),
        std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "Dispensing"), 
        std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "Dispensing")
      );

      opcua::services::createMonitoredItemDataChangeAsync(
        cli,
        res.subscriptionId(),
        opcua::ReadValueId(initiate_id, opcua::AttributeId::Value),
        opcua::MonitoringMode::Reporting,
        monitoring_params,
        std::bind(&DispenserStationNode::initiate_cb, this, _1, _2, _3),
        std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "Initiate"), 
        std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "Initiate")
      );

      create_mon_item_async(res, running_id, "Running", std::shared_ptr<bool>(status_, &status_->running));
      create_mon_item_async(res, paused_id, "Paused", std::shared_ptr<bool>(status_, &status_->paused));
      create_mon_item_async(res, error_id, "Error", std::shared_ptr<bool>(status_, &status_->error));

      for (size_t i = 0; i < NO_OF_UNITS; i++)
      {
        const uint8_t id = i + 1;

        opcua::services::createMonitoredItemDataChangeAsync(
          cli,
          res.subscriptionId(),
          opcua::ReadValueId(bin_open_req_id[id], opcua::AttributeId::Value),
          opcua::MonitoringMode::Reporting,
          monitoring_params,
          std::bind(&DispenserStationNode::open_close_req_cb, this, _1, _2, _3, bin_open_req_id[id], bin_opened_id[id]),
          std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "BinOpen"), 
          std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "BinOpen")
        );
        opcua::services::createMonitoredItemDataChangeAsync(
          cli,
          res.subscriptionId(),
          opcua::ReadValueId(bin_close_req_id[id], opcua::AttributeId::Value),
          opcua::MonitoringMode::Reporting,
          monitoring_params,
          std::bind(&DispenserStationNode::open_close_req_cb, this, _1, _2, _3, bin_close_req_id[id], bin_closed_id[id]),
          std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "BinClose"), 
          std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "BinClose")
        );
        opcua::services::createMonitoredItemDataChangeAsync(
          cli,
          res.subscriptionId(),
          opcua::ReadValueId(baffle_open_req_id[id], opcua::AttributeId::Value),
          opcua::MonitoringMode::Reporting,
          monitoring_params,
          std::bind(&DispenserStationNode::open_close_req_cb, this, _1, _2, _3, baffle_open_req_id[id], baffle_opened_id[id]),
          std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "BinClose"), 
          std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "BinClose")
        );
        opcua::services::createMonitoredItemDataChangeAsync(
          cli,
          res.subscriptionId(),
          opcua::ReadValueId(baffle_close_req_id[id], opcua::AttributeId::Value),
          opcua::MonitoringMode::Reporting,
          monitoring_params,
          std::bind(&DispenserStationNode::open_close_req_cb, this, _1, _2, _3, baffle_close_req_id[id], baffle_opened_id[id]),
          std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, "BinClose"), 
          std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, "BinClose")
        );
      }

      for (size_t i = 0; i < NO_OF_UNITS; i++)
      {
        const uint8_t id = i + 1;
        auto unit_status = std::shared_ptr<DispenserUnitStatus>(status_, &status_->unit_status[i]);

        create_mon_item_async(res, unit_lack_id[id], "Unit" + std::to_string(id) + "Lack", std::shared_ptr<bool>(unit_status, &unit_status->lack));

        create_mon_item_async(res, bin_opened_id[id], "Bin" + std::to_string(id) + "Opened", std::shared_ptr<bool>(unit_status, &unit_status->bin_opened));
        create_mon_item_async(res, bin_closed_id[id], "Bin" + std::to_string(id) + "Closed", std::shared_ptr<bool>(unit_status, &unit_status->bin_closed));

        create_mon_item_async(res, baffle_opened_id[id], "Baffle" + std::to_string(id) + "Opened", std::shared_ptr<bool>(unit_status, &unit_status->baffle_opened));
        create_mon_item_async(res, baffle_closed_id[id], "Baffle" + std::to_string(id) + "Closed", std::shared_ptr<bool>(unit_status, &unit_status->baffle_closed));
      }
    }
  );

  RCLCPP_DEBUG(this->get_logger(), "%s is working", __FUNCTION__);
}

void DispenserStationNode::create_mon_item_async(
  const opcua::CreateSubscriptionResponse &res, 
  const opcua::NodeId &id, 
  const std::string name, 
  std::shared_ptr<bool> ptr) 
{
  opcua::MonitoringParametersEx monitoring_params{};
  monitoring_params.samplingInterval = 250.0;
  monitoring_params.queueSize = 1;
  monitoring_params.discardOldest = true;

  opcua::services::createMonitoredItemDataChangeAsync(
    cli,
    res.subscriptionId(),
    opcua::ReadValueId(id, opcua::AttributeId::Value),
    opcua::MonitoringMode::Reporting,
    monitoring_params,
    std::bind(&DispenserStationNode::general_bool_cb, this, _1, _2, _3, name, ptr),
    std::bind(&DispenserStationNode::monitored_item_deleted_cb, this, _1, _2, name), 
    std::bind(&DispenserStationNode::monitored_item_created_cb, this, _1, name)
  );
}

void DispenserStationNode::sub_status_change_cb(uint32_t sub_id, opcua::StatusChangeNotification &notification)
{
  (void) notification;
  RCLCPP_INFO(this->get_logger(), ">>>> Subscription status change: %d", sub_id);
  std::this_thread::sleep_for(1s);
}

void DispenserStationNode::sub_deleted_cb(uint32_t sub_id)
{
  RCLCPP_INFO(this->get_logger(), ">>>> Subscription deleted: %d", sub_id);
  std::this_thread::sleep_for(1s);
}

void DispenserStationNode::monitored_item_deleted_cb(uint32_t sub_id, uint32_t mon_id, std::string name)
{
  RCLCPP_INFO(this->get_logger(), ">>>> %s Monitored Item deleted:", name.c_str());
  RCLCPP_INFO(this->get_logger(), ">>>> - subscription id: %d", sub_id);
  RCLCPP_INFO(this->get_logger(), ">>>> - monitored item id: %d", mon_id);
}

void DispenserStationNode::monitored_item_created_cb(opcua::MonitoredItemCreateResult& result, std::string name)
{
  RCLCPP_DEBUG(this->get_logger(), ">>>> %s Monitored Item created:", name.c_str());
  RCLCPP_DEBUG(this->get_logger(), ">>>> - status code: %s", std::to_string(result.statusCode()).c_str());
  RCLCPP_DEBUG(this->get_logger(), ">>>> - monitored item id: %s", std::to_string(result.monitoredItemId()).c_str());
}

void DispenserStationNode::disconnected_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_ERROR(this->get_logger(), "Disconnected to opcua server: %s:%s", ip_.c_str(), port_.c_str());
}

void DispenserStationNode::connected_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(this->get_logger(), "Connected to opcua server: %s:%s", ip_.c_str(), port_.c_str());
}

void DispenserStationNode::session_activated_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  create_sub_async();
  RCLCPP_INFO(this->get_logger(), "Session activated: %s:%s", ip_.c_str(), port_.c_str());
}

void DispenserStationNode::session_closed_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_ERROR(this->get_logger(), "Session closed: %s:%s", ip_.c_str(), port_.c_str());
}

void DispenserStationNode::inactive_cb(void)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_ERROR(this->get_logger(), "Inactive: %s:%s", ip_.c_str(), port_.c_str());
}
