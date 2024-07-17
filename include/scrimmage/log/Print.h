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
 * A long description.
 * ---------------------------------------------------------------------------
 */

#ifndef INCLUDE_SCRIMMAGE_LOG_PRINT_H_
#define INCLUDE_SCRIMMAGE_LOG_PRINT_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <ostream>
#include <string>

namespace scrimmage {

class Time;
using TimePtr = std::shared_ptr<Time>;

struct PrintEnums {
  enum class WARN_LEVEL : char { DEV = 0, INFO = 1, WARNING = 2, ERROR = 3 };
  enum class WRITE_TO : char { BOTH = 0, FILE_ONLY = 1, CONSOLE_ONLY = 2 };
};

struct PrintData {
  PrintData() : time_(std::nan("")), name_("UnknownClass"), entity_id_(-1) {}
  double time_;
  std::string name_;
  int entity_id_;
};

class Print {
 public:
  Print()
      : time_(nullptr),
        log_dir_(""),
        output_flag_(PrintEnums::WRITE_TO::BOTH) {}

  void init(TimePtr &time, std::string log_dir);
  void init(TimePtr &time, std::string log_dir, PrintEnums::WRITE_TO flag);
  void close();
  void flush();

  void printDev(PrintData &data, std::string outmessage);
  void printInfo(PrintData &data, std::string outmessage);
  void printWarning(PrintData &data, std::string outmessage);
  void printError(PrintData &data, std::string outmessage);

  void printDev(EntityPlugin &caller, std::string outmessage);
  void printInfo(EntityPlugin &caller, std::string outmessage);
  void printWarning(EntityPlugin &caller, std::string outmessage);
  void printError(EntityPlugin &caller, std::string outmessage);

 protected:
  static const char *getWarningLevel(PrintEnums::WARN_LEVEL e);

  static std::string formatTime(double time);
  static std::string formatMsg(PrintEnums::WARN_LEVEL level, PrintData &data,
                               std::string msg);

  void print(std::ostream &stream, PrintEnums::WARN_LEVEL level,
             EntityPlugin &caller, std::string msg);
  void print(std::ostream &stream, PrintEnums::WARN_LEVEL level,
             PrintData &data, std::string msg);

 private:
  TimePtr time_;
  std::string log_dir_;
  std::ofstream log_stream_;
  PrintEnums::WRITE_TO output_flag_;

  bool write_file();
  bool write_console();
};

using PrintPtr = std::shared_ptr<Print>;

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_LOG_PRINT_H_
