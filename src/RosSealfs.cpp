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

#include <ctype.h>
#include <memory>
#include <thread>
#include <exception>
#include <utility>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "LogTopic.hpp"
#include "RosSealfs.hpp"
#include "DecoderFactory.hpp"
#include "Decoder.hpp"

using std::placeholders::_1;

namespace rossealfs
{

void RosSealfs::parameters_init()
{
  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.name = "mountpoint";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  desc.description = "Mount point for sealfs in the node";
  this->declare_parameter("mountpoint", "/var/log/", desc, false);

  desc.name = "categories";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  desc.description = "Sealfs logging categories";
  std::vector<std::string> arr = { "all" };
  this->declare_parameter("categories", arr, desc, false);

  desc.name = "files";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  desc.description = "Sealfs log files";
  arr = { "/rossealfs-all.log" };
  this->declare_parameter("files", arr, desc, false);

  desc.name = "types";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  desc.description = "Sealfs topic type";
  arr = { "std_msgs/String" };
  this->declare_parameter("types", arr, desc, false);
}

int RosSealfs::check_path(std::string s)
{
  return s[0] == '/';
}

int RosSealfs::check_category_name(std::string s)
{
  for (int i = 0; i < static_cast<int>(s.length()); i++)
  {
    if(!isalnum(s[i]))
    {
      return 0;
    }
  }
  return 1;
}

void RosSealfs::parameters_read()
{
  rclcpp::Parameter point;
  rclcpp::Parameter cats;
  rclcpp::Parameter files;
  rclcpp::Parameter types;

  point = get_parameter("mountpoint");
  cats = get_parameter("categories");
  files = get_parameter("files");
  types = get_parameter("types");

  auto catsarr = cats.as_string_array();
  auto filesarr = files.as_string_array();
  auto typesarr = types.as_string_array();

  if (catsarr.size() != filesarr.size() || filesarr.size() != typesarr.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Wrong parameters: sizes do not match");
    throw std::string("wrong parameters: sizes do not match");
  }
  this->mountpoint = point.as_string();
  if (!this->check_path(this->mountpoint))
  {
    throw std::string("mounpoint is not an absolute path");
  }
  for (int i = 0; i < static_cast<int>(catsarr.size()); i++)
  {
    if (!this->check_path(filesarr[i]))
    {
      throw std::string("log file is not an absolute path");
    }
    if (!this->check_category_name(catsarr[i]))
    {
      throw std::string("bad category name (only alphanumeric chars allowed)");
    }
    try
    {
      auto lt = std::make_shared<LogTopic>(this->mountpoint, catsarr[i], filesarr[i], typesarr[i]);
      this->topics.push_back(lt);
    }
    catch (std::string e)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Exception (%s) can't make LogTopic: "
                   "mountpoint: %s file: %s",
                   e.c_str(), this->mountpoint.c_str(), filesarr[i].c_str());
      throw e;
    }
  }
}

RosSealfs::RosSealfs() : Node("rossealfs")
{
  parameters_init();
  parameters_read();

  auto str = std::string("");
  for (auto elem : this->topics)
  {
    str = " " + str + elem.get()->to_string();
  }
  RCLCPP_INFO(this->get_logger(), "mountpoint: %s topics:%s",
              this->mountpoint.c_str(), str.c_str());
  for (auto elem : this->topics)
  {
    auto cat = elem.get()->get_category();
    auto topicname = std::string("/sealfs/") + cat;
    auto type = elem.get()->get_type();
    auto logtopic = elem;
    auto that = this;
    auto lambda = [that, type, topicname, logtopic](
              std::shared_ptr<rclcpp::SerializedMessage> msg) {
      auto f = std::make_shared<DecoderFactory>();
      auto decoder = f.get()->produce(type, msg, logtopic);
      if(decoder.get()->log() < 0)
      {
        RCLCPP_ERROR(that->get_logger(), "error logging message, type: %s", type.c_str());
      }
    };
    rclcpp::QoS qos = rclcpp::QoS(10);
    auto s = this->create_generic_subscription(topicname, type, qos, lambda);
    this->subscriptions.push_back(s);
    RCLCPP_INFO(this->get_logger(), "subscribed to: %s", topicname.c_str());
  }
  this->timer = rclcpp::create_timer(this,
                                this->get_clock(),
                                std::chrono::seconds(SyncFreqSecs),
                                std::bind(&RosSealfs::sync, this));
}

void RosSealfs::sync()
{
  for (auto topic : this->topics)
  {
    if (topic->sync() == -1)
    {
      auto s = topic->to_string();
      RCLCPP_ERROR(this->get_logger(), "error syncing LogTopic %s",  s.c_str());
    }
  }
}

}  // namespace rossealfs
