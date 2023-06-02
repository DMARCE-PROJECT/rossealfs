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


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "LogTopic.hpp"

namespace rossealfs
{

enum{
  BufSize = 32 * 1024,
};

LogTopic::LogTopic(std::string mp, std::string c, std::string f, std::string t)
{
  int sz;

  this->category = c;
  this->file = f;
  this->type = t;
  auto path = mp + f;
  int fd = open(path.c_str(), O_WRONLY | O_APPEND | O_CREAT, 0600);
  if (fd < 0)
  {
    throw std::string("open syscall failed, path: ") + path;
  }
  this->stream = fdopen(fd, "a");
  if (this->stream == NULL)
  {
    throw std::string("fdopen failed, path: ") + path;
  }
  sz = BUFSIZ;
  if(BufSize > sz)
  {
    sz = BufSize;
  }
  this->buf = reinterpret_cast<char *>(malloc(sz));
  if (this->buf == NULL)
  {
    throw std::string("OOM");
  }
  if (setvbuf(this->stream, this->buf, _IOFBF, sz) != 0)
  {
    free(this->buf);
    throw std::string("buffer can't be configured");
  }
}

LogTopic::~LogTopic()
{
  fclose(this->stream);
  free(this->buf);
}

std::string LogTopic::to_string()
{
  auto s = "[category: " + this->category + ", file: ";
  return s + this->file + ", type: " + this->type + "]";
}

std::string LogTopic::get_category()
{
  return this->category;
}

std::string LogTopic::get_type()
{
  return this->type;
}

int LogTopic::sync()
{
  if(fflush(this->stream) != 0)
  {
    return -1;
  }
  return 0;
}

int LogTopic::log_write(void * buf, int64_t len)
{
  if (fwrite(buf, static_cast<size_t>(len), 1, this->stream) != 1)
  {
    return -1;
  }
  return 0;
}

}  // namespace rossealfs
