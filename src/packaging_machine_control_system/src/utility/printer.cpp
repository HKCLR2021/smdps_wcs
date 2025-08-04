#include "packaging_machine_control_system/packaging_machine_node.hpp"

// ===================================== printer =====================================

void PackagingMachineNode::init_printer(void)
{
  printer_.reset();
  printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial,
    printer_config_->port);

  RCLCPP_INFO(this->get_logger(), "printer initialized");
}

void PackagingMachineNode::init_printer_config(void)
{
  printer_->configure(printer_config_->endpoint_in, printer_config_->endpoint_out, printer_config_->timeout);

  std::string size_str = "75 mm," + std::to_string(status_->package_length) + " mm";
  
  printer_->addDefaultConfig("SIZE", size_str);
  printer_->addDefaultConfig("GAP", "0 mm, 0 mm");
  printer_->addDefaultConfig("SPEED", "1");
  printer_->addDefaultConfig("DENSITY", "15");
  printer_->addDefaultConfig("DIRECTION", "0,0");
  printer_->addDefaultConfig("REFERENCE", "0,0");
  printer_->addDefaultConfig("OFFSET", "0 mm");
  printer_->addDefaultConfig("SHIFT", "-177"); // 15 mm
  printer_->addDefaultConfig("SET", "TEAR OFF");
  printer_->addDefaultConfig("SET", "REWIND OFF");
  printer_->addDefaultConfig("SET", "PEEL OFF");
  printer_->addDefaultConfig("SET", "CUTTER OFF");
  printer_->addDefaultConfig("SET", "PARTIAL_CUTTER OFF");
  printer_->addDefaultConfig("CLS");
}

std::string PackagingMachineNode::add_drug_space(std::string drug_str)
{
  for_each(drug_str.begin(), drug_str.end(), [](char& c) {
      c = toupper(c);
  });

  if (drug_str.length() > MAX_DRUG_LEN) 
  {
    drug_str = drug_str.substr(0, MAX_DRUG_LEN);
    return drug_str;
  }

  uint8_t num_of_space = 0;
  while (drug_str.length() < MAX_DRUG_LEN)
  {
    drug_str.insert(drug_str.end() - 1, ' ');
    num_of_space++;
  }
  RCLCPP_INFO(this->get_logger(), "Added %d number of space to drug name", num_of_space);

  // if (drug_str.find("TABLET") != std::string::npos) 
  // {
  //   drug_str.append(" TAB");
  //   RCLCPP_INFO(this->get_logger(), "Added TAB unit");
  // } 
  // else if (drug_str.find("CAPSULE") != std::string::npos) 
  // {
  //   drug_str.append(" CAP");
  //   RCLCPP_INFO(this->get_logger(), "Added CAP unit");
  // } 
  // else
  // {
  //   drug_str.append(" EA");
  //   RCLCPP_INFO(this->get_logger(), "Added EA unit");
  // }

  return drug_str;
}

smdps_msgs::msg::PackageInfo PackagingMachineNode::create_printer_info_temp(void)
{
  PackageInfo msg;

  msg.cn_name = "Centre A";
  msg.en_name = "Chan Tai Man";

  if (printer_test_date_index < printer_test_date.size())
    msg.date = printer_test_date[printer_test_date_index];
  else
    msg.date = "out of range";

  if (printer_test_meal_index < printer_test_meal.size())
    msg.time = printer_test_meal[printer_test_meal_index++];
  else
    msg.time = "out of range";
  
  msg.qr_code = "https://www.hkclr.hk";

  std::string drug;
  drug = "PARACETAMOL TABLET 500MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "HYDRALAZINE HCI TABLET 25MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "FAMOTIDINE TABLET 20MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "COLCINA TABLET 0.5MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "DIMETHYLPOLYSILOXANE TABLET 40MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  drug = "SENNA TABLET 7.5MG 1";
  msg.drugs.push_back(add_drug_space(drug));
  // msg.drugs.push_back("12345678901234567890123456789012345678901234567890");

  RCLCPP_INFO(this->get_logger(), add_drug_space(drug).c_str());


  if (printer_test_meal_index >= printer_test_meal.size())
  {
    printer_test_meal_index = 0; 
    printer_test_date_index++;
  }
  
  if (printer_test_date_index >= printer_test_date.size())
    printer_test_date_index = 0;

  return msg;
}

std::vector<std::string> PackagingMachineNode::get_print_label_cmd(PackageInfo msg)
{
  std::vector<std::string> cmds{};

  if (!msg.en_name.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Add a english name: %s", msg.en_name.c_str());

    // std::string gbk_cn = printer_->convert_utf8_to_gbk(msg.cn_name);
    // std::string cn = "TEXT 240,180,\"3\",0,2,2,\"" + gbk_cn + "\"";
    std::string cn_str;

    if (printer_->is_valid_utf8(msg.cn_name))
      cn_str = msg.cn_name;
    else
    {
      cn_str = printer_->convert_utf8_to_gbk(msg.cn_name);
    }
    std::string cn = "TEXT 150,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + cn_str + "\"";
    cmds.emplace_back(cn);
    // std::string en = "TEXT 150,225,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.en_name + "\"";
    std::string en = "TEXT 190,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.en_name + "\"";
    cmds.emplace_back(en);
    // std::string d = "TEXT 200,575,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.date + "\"";
    std::string d = "TEXT 230,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.date + "\"";
    cmds.emplace_back(d);
    // std::string t = "TEXT 250,575,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.time + "\"";
    std::string t = "TEXT 270,550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + msg.time + "\"";
    cmds.emplace_back(t);
    // std::string q = "QRCODE 200,225,H,8,A,270,\"" + msg.qr_code + "\"";
    std::string q = "QRCODE 150,275,H,5,A,270,\"" + msg.qr_code + "\"";
    cmds.emplace_back(q);
    for (size_t index = 0; index < msg.drugs.size(); index++) 
    {
      std::string utf_md = msg.drugs[index];
      // std::string gbk_md = printer_->convert_utf8_to_gbk(utf_md);
      int x = 400 + index * 40;
      std::string x_label = std::to_string(x);
      std::string m = "TEXT " + x_label + ",550,\"" + std::to_string(printer_font_) + "\",270,1,1,\"" + utf_md + "\"";
      cmds.emplace_back(m);
    }
  }

  cmds.emplace_back("PRINT 1,1");
  // cmds.emplace_back("EOP");
  RCLCPP_DEBUG(this->get_logger(), "printer commands are ready");
  return cmds;
}