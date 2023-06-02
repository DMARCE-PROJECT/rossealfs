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

namespace rossealfs
{

Decoder::Decoder(std::shared_ptr<rclcpp::SerializedMessage> themsg,
                  std::shared_ptr<LogTopic> lt):
        msg(themsg),
        logtopic(lt)
{
}

int Decoder::log()
{
  int64_t len;
  void * buf;
  void * p;

  buf = reinterpret_cast<void*>(this->msg.get()->get_rcl_serialized_message().buffer);
  len = (int64_t)this->msg.get()->get_rcl_serialized_message().buffer_length;
  p = reinterpret_cast<void*>(&len);
  if (this->logtopic.get()->log_write(p, sizeof(len)) < 0)
  {
   return -1;
  }
  return this->logtopic->log_write(buf, len);
}

}  // namespace rossealfs
