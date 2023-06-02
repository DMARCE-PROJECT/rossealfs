// Copyright 2023 Enrique Soriano <enrique.soriano@urjc.es>
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#include <memory>
#include <string>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "Decoder.hpp"
#include "RosoutDecoder.hpp"

namespace rossealfs
{

RosoutDecoder::RosoutDecoder(std::shared_ptr<rclcpp::SerializedMessage> themsg,
            std::shared_ptr<LogTopic> lt) : Decoder(themsg, lt)
{
}

enum{
  DEBUG = 10,
  INFO = 20,
  WARN = 30,
  ERROR = 40,
  FATAL = 50,
};

int RosoutDecoder::log()
{
  int64_t len;
  void * buf;
  rclcpp::Serialization<rcl_interfaces::msg::Log> serializer;
  rcl_interfaces::msg::Log deserialized;

  serializer.deserialize_message(this->msg.get(), &deserialized);
  auto logline = std::string("level: ");
  switch(static_cast<int>(deserialized.level)){
  case DEBUG:
    logline += std::string("DEBUG");
    break;
  case INFO:
    logline += std::string("INFO");
    break;
  case WARN:
    logline += std::string("WARN");
    break;
  case ERROR:
    logline += std::string("ERROR");
    break;
  case FATAL:
    logline += std::string("FATAL");
    break;
  default:
    logline += std::to_string(deserialized.level) + std::string("(BUG?)");
  }
  logline += std::string(", name: ") + deserialized.name;
  logline += std::string(", msg: ") + deserialized.msg;
  logline += std::string(", file: ") + deserialized.file;
  logline += std::string(", function: ") + deserialized.function;
  logline += std::string(", line: ") +
            std::to_string(static_cast<int>(deserialized.line));
  logline += std::string("\n");
  len = static_cast<int64_t>(strlen(logline.c_str()));
  buf = reinterpret_cast<void*>(const_cast<char*>(logline.c_str()));
  return this->logtopic.get()->log_write(buf, len);
}

}  // namespace rossealfs
