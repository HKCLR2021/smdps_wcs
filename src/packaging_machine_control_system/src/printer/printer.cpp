//
// Created by Dafan Wang on 7/26/24.
// Modified by Sam Kwok on 21 Nov 2024.
//

#include "printer/config.h"
#include "printer/printer.h"
#include "printer/libusbxx.hpp"

Printer::Printer(uint16_t vendor_id, uint16_t product_id, std::string_view serial_num, uint8_t port)
  : usb_(std::make_unique<libusbxx>()) 
{
  usb_->openDevice(vendor_id, product_id, serial_num, port);
}

Printer::Printer(uint16_t vendor_id, uint16_t product_id, std::string_view serial_num)
  : usb_(std::make_unique<libusbxx>()) 
{
  usb_->openDevice(vendor_id, product_id, serial_num);
}

Printer::Printer(uint16_t vendor_id, uint16_t product_id)
  : usb_(std::make_unique<libusbxx>()) 
{
  usb_->openDevice(vendor_id, product_id);
}

Printer::~Printer() = default;

void Printer::configure(uint8_t endpoint_in, uint8_t endpoint_out, uint32_t timeout) 
{
  endpoint_in_ = endpoint_in;
  endpoint_out_ = endpoint_out;
  timeout_ = timeout;
}

void Printer::addDefaultConfig(const std::string &name, const std::string &config) 
{
  default_cmd_.try_emplace(name, config);
}

void Printer::addDefaultConfig(const std::string &cmd) 
{
  default_cmd_.try_emplace(cmd, "");
}

bool Printer::updateDefaultConfig(const std::string &name, const std::string &config) 
{
  // if (default_cmd_.contains(name)) {
  if (default_cmd_.find(name) != default_cmd_.end()) {
    default_cmd_[name] = config;
    return true;
  }

  return false;
}

void Printer::runTask(const std::vector<std::string> &cmds) 
{
  for (auto &[name, config]: default_cmd_) {
    // usb_->bulkTransfer(endpoint_out_, std::format("{} {}\r\n", name, config), timeout_);
    usb_->bulkTransfer(endpoint_out_, name + " " + config + "\r\n", timeout_);
  }

  for (auto command: cmds) {
    // usb_->bulkTransfer(endpoint_out_, std::format("{}\r\n", command), timeout_);
    usb_->bulkTransfer(endpoint_out_, command + "\r\n", timeout_);
  }
}

std::string Printer::convert_utf8_to_gbk(const std::string &utf8_string) 
{
  // 设置转换
  iconv_t cd = iconv_open("GBK", "UTF-8");
  if (cd == (iconv_t)(-1)) {
      perror("iconv_open failed");
      return "";
  }

  // 准备输入和输出
  char *in_buf = const_cast<char*>(utf8_string.c_str());
  size_t in_bytes_left = utf8_string.size();
  
  // 预分配输出缓冲区，GBK 编码可能会稍大
  size_t out_buf_size = in_bytes_left * 2; // 预留空间
  char *out_buf = new char[out_buf_size];
  char *out_ptr = out_buf;
  size_t out_bytes_left = out_buf_size;

  // 执行转换
  size_t result = iconv(cd, &in_buf, &in_bytes_left, &out_ptr, &out_bytes_left);
  
  if (result == (size_t)(-1)) {
    perror("iconv failed");
    delete[] out_buf;
    iconv_close(cd);
    return "";
  }

  // 计算转换后的字符串长度
  size_t converted_length = out_buf_size - out_bytes_left;
  
  // 生成输出字符串
  std::string gbk_string(out_buf, converted_length);

  // 清理
  delete[] out_buf;
  iconv_close(cd);

  return gbk_string;
}

bool Printer::is_valid_utf8(const std::string& s)
{
  const unsigned char* bytes = reinterpret_cast<const unsigned char*>(s.data());
  size_t len = s.length();
  size_t i = 0;

  while (i < len) 
  {
    unsigned char byte = bytes[i];
    if (byte < 0x80) 
    { 
      // 1-byte character (ASCII)
      i++;
    } 
    else if ((byte & 0xE0) == 0xC0) 
    { 
      // 2-byte character
      if (i + 1 >= len || (bytes[i+1] & 0xC0) != 0x80) 
        return false;
      // Check for overlong encoding (U+0080 to U+07FF)
      if (byte < 0xC2) 
        return false;
      i += 2;
    } 
    else if ((byte & 0xF0) == 0xE0) 
    { 
      // 3-byte character
      if (i + 2 >= len || (bytes[i+1] & 0xC0) != 0x80 || (bytes[i+2] & 0xC0) != 0x80) 
        return false;
      // Check for overlong encoding (U+0800 to U+FFFF) and surrogate range
      if (byte == 0xE0 && bytes[i+1] < 0xA0) 
        return false;
      if (byte == 0xED && bytes[i+1] >= 0xA0) 
        return false; // Surrogate range
      i += 3;
    } 
    else if ((byte & 0xF8) == 0xF0) 
    { 
      // 4-byte character
      if (i + 3 >= len || (bytes[i+1] & 0xC0) != 0x80 || (bytes[i+2] & 0xC0) != 0x80 || (bytes[i+3] & 0xC0) != 0x80) 
        return false;
      // Check for overlong encoding (U+10000 to U+10FFFF) and valid range
      if (byte == 0xF0 && bytes[i+1] < 0x90) 
        return false;
      if (byte == 0xF4 && bytes[i+1] >= 0x90) 
        return false;
      i += 4;
    } 
    else 
    { 
      // Invalid leading byte
      return false;
    }
  }
  return true;
}