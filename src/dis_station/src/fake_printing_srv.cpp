#include "rclcpp/rclcpp.hpp"

#include "smdps_msgs/srv/printing_order.hpp"
#include "smdps_msgs/msg/printing_info.hpp"
#include "smdps_msgs/msg/patient_info.hpp"
#include "smdps_msgs/msg/material_box_slot.hpp"
#include "smdps_msgs/msg/dispensing_detail.hpp"
#include "smdps_msgs/msg/drug.hpp"

using PrintingOrder = smdps_msgs::srv::PrintingOrder;

// This node is used for creating a PrintingOrder Service for simulation.
class PrintingService : public rclcpp::Node
{
public:
  PrintingService() : Node("fake_printing_srv")
  {
    service_ = this->create_service<PrintingOrder>(
      "printing_order",
      std::bind(&PrintingService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "fake_printing_srv is up.");
  }

private:
  void handle_service(const std::shared_ptr<PrintingOrder::Request> request,
                      std::shared_ptr<PrintingOrder::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received material_box_id: %d", request->material_box_id);

    response->info.patient_info.name = "Testing";
    response->info.patient_info.name_ch = "HKCLR";
    response->info.qr_code = "https://www.hkclr.hk";

    for (size_t i = 0; i < 28; ++i)
    {
      smdps_msgs::msg::Drug drug;
      drug.drug_id = "id " + std::to_string(i);
      drug.amount = 1;
      drug.name = "fake drug"; 
      response->info.slots[i].drugs.push_back(drug);
    }

    response->success = true;
    response->message = "";

    RCLCPP_INFO(this->get_logger(), "Sending response: success=%s, message='%s'", 
                response->success ? "true" : "false", response->message.c_str());
  }

  rclcpp::Service<PrintingOrder>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrintingService>());
  rclcpp::shutdown();
  return 0;
}