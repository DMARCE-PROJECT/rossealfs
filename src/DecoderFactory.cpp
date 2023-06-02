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
#include "DecoderFactory.hpp"
#include "Decoder.hpp"
#include "StringDecoder.hpp"
#include "RosoutDecoder.hpp"

namespace rossealfs
{

std::shared_ptr<Decoder> DecoderFactory::produce(std::string rostype,
         std::shared_ptr<rclcpp::SerializedMessage> themsg,
         std::shared_ptr<LogTopic> lt)
{
  if(rostype == "std_msgs/String")
  {
    return std::make_shared<StringDecoder>(themsg, lt);
  } else if(rostype == "rcl_interfaces/msg/Log") {
    return std::make_shared<RosoutDecoder>(themsg, lt);
  } else {
    return std::make_shared<Decoder>(themsg, lt);
  }
}

}  // namespace rossealfs
