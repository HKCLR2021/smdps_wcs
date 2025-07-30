#include "packaging_machine_control_system/packaging_machine_node.hpp"

uint32_t PackagingMachineNode::read_ribbon(std::string type)
{
  const char* mem_path = std::getenv("PKG_MAC_MEMORY_PATH");
  if (!mem_path)
  {
    RCLCPP_ERROR(this->get_logger(), "PKG_MAC_MEMORY_PATH does not existed!!!");
    return 0;
  }

  std::string filename = std::string(mem_path) + "/remain_" + type + "_" + std::to_string(status_->packaging_machine_id);
  std::ifstream read_file(filename);

  if (!read_file.is_open()) 
  {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  std::string length;
  if (!std::getline(read_file, length) || length.empty()) 
  {
    throw std::runtime_error("File is empty or invalid: " + filename);
  }

  try 
  {
    size_t pos;
    int value = std::stoi(length, &pos);
    if (pos != length.size() || value < 0 || value > INT32_MAX) 
    {
      throw std::runtime_error("Invalid ribbon length in file: " + filename);
    }
    return static_cast<uint32_t>(value);
  }
  catch (const std::exception& e) 
  {
    throw std::runtime_error("Failed to parse ribbon length: " + std::string(e.what()));
  }
}

void PackagingMachineNode::write_ribbon(std::string type, uint32_t ribbon_length)
{
  const char* mem_path = std::getenv("PKG_MAC_MEMORY_PATH");
  if (!mem_path)
  {
    RCLCPP_ERROR(this->get_logger(), "PKG_MAC_MEMORY_PATH does not existed!!!");
    return;
  }
  
  std::string filename = std::string(mem_path) + "/remain_" + type + "_" + std::to_string(status_->packaging_machine_id);
  std::ofstream file(filename);

  if (!file.is_open()) 
  {
    throw std::runtime_error("Failed to open file for writing: " + filename);
  }

  file << std::to_string(ribbon_length);

  if (!file.good()) 
  {
    throw std::runtime_error("Failed to write to file: " + filename);
  }

  file.close();
}
