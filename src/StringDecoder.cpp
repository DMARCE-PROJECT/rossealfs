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
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Decoder.hpp"
#include "StringDecoder.hpp"

namespace rossealfs
{

StringDecoder::StringDecoder(std::shared_ptr<rclcpp::SerializedMessage> themsg,
            std::shared_ptr<LogTopic> lt) : Decoder(themsg, lt)
{
}


int StringDecoder::log()
{
  int64_t len;
  void * buf;
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  std_msgs::msg::String deserialized;

  serializer.deserialize_message(this->msg.get(), &deserialized);
  auto logline = deserialized.data + std::string("\n");
  len = static_cast<int64_t>(strlen(logline.c_str()));
  buf = reinterpret_cast<void*>(const_cast<char*>(logline.c_str()));
  return this->logtopic.get()->log_write(buf, len);
}

}  // namespace rossealfs
