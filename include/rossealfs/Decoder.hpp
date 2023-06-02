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

#ifndef ROSSEALFS__DECODER_HPP_
#define ROSSEALFS__DECODER_HPP_

#include <memory>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "LogTopic.hpp"

namespace rossealfs
{

class Decoder
{
public:
  Decoder(std::shared_ptr<rclcpp::SerializedMessage> themsg,
              std::shared_ptr<LogTopic> lt);
  virtual int log();

protected:
  std::shared_ptr<rclcpp::SerializedMessage> msg;
  std::shared_ptr<LogTopic> logtopic;
};
}  // namespace rossealfs

#endif  // ROSSEALFS__DECODER_HPP_
