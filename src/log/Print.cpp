/*
 * ---------------------------------------------------------------------------
 * @section LICENSE
 *
 * Copyright (c) 2020 Georgia Tech Research Institute (GTRI)
 *               All Rights Reserved
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * ---------------------------------------------------------------------------
 * @file filename.ext
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @version 1.0
 * ---------------------------------------------------------------------------
 * @brief A brief description.
 *
 * @section DESCRIPTION
 * Example usage
 *
 * // Required Includes:
 *   #include <scrimmage/entity/Entity.h>
 *   #include <scrimmage/log/Print.h>
 *
 * // In a Plugin (any type):
 * In the _step() method, for example:
 *   this->parent()->printer()->printInfo(*this, "Printing from my plugin!");
 *
 *   // or
 *
 *   // requires: #include <sstream>
 *   std::ostringstream o;
 *   o << "Testing:" << std::endl;
 *   o << "    multiline " << std::endl;
 *   o << "    prints " << std::endl;
 *   this->parent()->printer()->printInfo(*this, o.str());
 *
 *
 * // In a lambda:
 *   auto mycallback = [=] (auto &somethingorother) {
 *                      ^
 *      // or
 *
 *   auto mycallback = [&,this] (auto &somethingorother) {
 *       //               ^^^^
 *       some_value_ = somethingorother->data;
 *       this->parent()->printer()->printDev(*this, "Somethingorother set!");
 *   };
 *
 * // In a utility class:
 *   'this' pointer of the calling plugin must be passed into the utility
 *   class/function/method and can be used as normal.
 *
 *
 * ---------------------------------------------------------------------------
 */

#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/log/Print.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <sstream>

namespace scrimmage {

void Print::init(TimePtr &time, std::string log_dir) {
  init(time, log_dir, PrintEnums::WRITE_TO::BOTH);
}
void Print::init(TimePtr &time, std::string log_dir,
                 PrintEnums::WRITE_TO flag) {
  time_ = time;
  log_dir_ = log_dir;
  output_flag_ = flag;

  if (write_file()) {
    std::ostringstream filename;
    filename << log_dir_ << '/' << "console.log";
    log_stream_.open(filename.str());
  }
}
void Print::close() {
  if (log_stream_) {
    log_stream_.close();
  }
  time_ = nullptr;
}
void Print::flush() {
  if (log_stream_) {
    log_stream_.flush();
  }
}

void Print::printDev(PrintData &data, std::string outmessage) {
  print(std::cout, PrintEnums::WARN_LEVEL::DEV, data, outmessage);
}
void Print::printInfo(PrintData &data, std::string outmessage) {
  print(std::cout, PrintEnums::WARN_LEVEL::INFO, data, outmessage);
}
void Print::printWarning(PrintData &data, std::string outmessage) {
  print(std::cout, PrintEnums::WARN_LEVEL::WARNING, data, outmessage);
}
void Print::printError(PrintData &data, std::string outmessage) {
  print(std::cerr, PrintEnums::WARN_LEVEL::ERROR, data, outmessage);
}

void Print::printDev(EntityPlugin &caller, std::string outmessage) {
  print(std::cout, PrintEnums::WARN_LEVEL::DEV, caller, outmessage);
}
void Print::printInfo(EntityPlugin &caller, std::string outmessage) {
  print(std::cout, PrintEnums::WARN_LEVEL::INFO, caller, outmessage);
}
void Print::printWarning(EntityPlugin &caller, std::string outmessage) {
  print(std::cerr, PrintEnums::WARN_LEVEL::WARNING, caller, outmessage);
}
void Print::printError(EntityPlugin &caller, std::string outmessage) {
  print(std::cerr, PrintEnums::WARN_LEVEL::ERROR, caller, outmessage);
}

const char *Print::getWarningLevel(PrintEnums::WARN_LEVEL e) {
  const std::map<PrintEnums::WARN_LEVEL, const char *> GetAsStrings{
      {PrintEnums::WARN_LEVEL::DEV, "DEV"},
      {PrintEnums::WARN_LEVEL::INFO, "INFO"},
      {PrintEnums::WARN_LEVEL::WARNING, "WARN"},
      {PrintEnums::WARN_LEVEL::ERROR, "ERROR"}};
  auto it = GetAsStrings.find(e);
  return it == GetAsStrings.end() ? GetAsStrings.begin()->second : it->second;
}
bool Print::write_file() {
  return ((output_flag_ == PrintEnums::WRITE_TO::BOTH) ||
          (output_flag_ == PrintEnums::WRITE_TO::FILE_ONLY));
}
bool Print::write_console() {
  return ((output_flag_ == PrintEnums::WRITE_TO::BOTH) ||
          (output_flag_ == PrintEnums::WRITE_TO::CONSOLE_ONLY));
}

std::string Print::formatTime(double time) {
  if (std::isnan(time)) {
    return "-----.---";
  } else {
    std::ostringstream o;
    o << std::internal << std::fixed << std::setprecision(3) << std::setw(9)
      << std::setfill('0') << time;
    return o.str();
  }
}

std::string Print::formatMsg(PrintEnums::WARN_LEVEL level, PrintData &data,
                             std::string msg) {
  std::ostringstream pre;
  std::ostringstream formattedmsg;
  std::string entitySection = "";

  // add entity_id if available
  if (data.entity_id_ > 0) {
    entitySection = "[Entity: " + std::to_string(data.entity_id_) + "]";
  }

  // create our prefix
  pre << formatTime(data.time_) << "s [" << data.name_ << "]" << entitySection
      << "[" << Print::getWarningLevel(level) << "]: ";

  // remove last character if it's a newline
  if (msg.back() == '\n') {
    msg.pop_back();
  }

  // iterate and place the prefix at the beginning of each newline
  formattedmsg << pre.str();
  for (auto i = msg.cbegin(); i != msg.cend(); ++i) {
    formattedmsg << *i;
    // add prefix after each newline
    if (*i == '\n') {
      formattedmsg << pre.str();
    }
  }

  return formattedmsg.str();
}

void Print::print(std::ostream &stream, PrintEnums::WARN_LEVEL level,
                  EntityPlugin &caller, std::string msg) {
  PrintData pd;
  pd.time_ = caller.getTime()->t();
  pd.name_ = caller.name();
  pd.entity_id_ = caller.parent()->id().id();

  print(stream, level, pd, msg);
}
void Print::print(std::ostream &stream, PrintEnums::WARN_LEVEL level,
                  PrintData &data, std::string msg) {
  std::ostringstream o;

  o << formatMsg(level, data, msg) << std::endl;

  if (write_file()) {
    log_stream_ << o.str();
  }  // write to log file
  if (write_console()) {
    stream << o.str();
  }  // write to console
}

}  // namespace scrimmage
