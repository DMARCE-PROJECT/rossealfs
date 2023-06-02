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

#ifndef ROSSEALFS__LOGTOPIC_HPP_
#define ROSSEALFS__LOGTOPIC_HPP_

#include <string>

namespace rossealfs
{

class LogTopic
{
public:
  LogTopic(std::string mp, std::string c, std::string f, std::string t);
  ~LogTopic();

  std::string to_string();
  std::string get_category();
  std::string get_type();
  int sync();
  int log_write(void * buf, int64_t len);

private:
  std::string category;
  std::string file;
  std::string type;
  FILE * stream;
  char * buf;
};

}  // namespace rossealfs

#endif  // ROSSEALFS__LOGTOPIC_HPP_
