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
 * @file Logger.h
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @version 1.0
 * ---------------------------------------------------------------------------
 * @brief Simple thread-safe logging with file/line capture.
 *
 * Usage with Logger class:
 *   scrimmage::Logger& log = scrimmage::Logger::instance();
 *   log.init_dir("/path/to/logs");  // creates scrimmage.log in that dir
 *   log.info("message", __FILE__, __LINE__);
 *
 * Usage with macros (recommended - auto-captures file/line):
 *   LOG_INFO("status update");
 *   LOG_INFO("value is " << x << " units");   // stream syntax for variables
 *   LOG_WARN("entity " << id << " has low health");
 *   LOG_ERROR("error message");
 *   LOG_ERROR("parse failed", exception);     // appends ": <exception.what()>"
 *
 * ---------------------------------------------------------------------------
 */

#ifndef INCLUDE_SCRIMMAGE_LOG_LOGGER_H_
#define INCLUDE_SCRIMMAGE_LOG_LOGGER_H_

#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace scrimmage {

/**
 * @brief Thread-safe singleton logger with file and console output.
 */
class Logger {
 public:
    /// Get the singleton instance
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    /// Initialize logging to scrimmage.log in the given directory
    void init_dir(const std::string& log_dir);

    /// Close the log file
    void close();

    /// Log an INFO message
    void info(const std::string& msg, const char* file = "", int line = 0);

    /// Log a WARN message
    void warn(const std::string& msg, const char* file = "", int line = 0);

    /// Log an ERROR message
    void error(const std::string& msg, const char* file = "", int line = 0);

    /// Log an ERROR message with exception details
    void error(const std::string& msg, const std::exception& ex,
               const char* file = "", int line = 0);

    /// Log a WARN message with exception details
    void warn(const std::string& msg, const std::exception& ex,
              const char* file = "", int line = 0);

 private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void init(const std::string& path);
    void write(const char* level, const char* file, int line, const std::string& msg);

    /// Buffered message for pre-init logging
    struct PendingMessage {
        std::string level;
        std::string file;
        int line;
        std::string msg;
        std::string formatted;  // Pre-formatted for output
    };

    std::ofstream file_;
    std::mutex mutex_;
    std::vector<PendingMessage> pending_;
    bool dir_initialized_{false};
};

}  // namespace scrimmage

// =============================================================================
// Convenience macros (auto-capture file and line)
// =============================================================================

#define LOG_INFO(expr) \
    do { \
        std::ostringstream _ss; \
        _ss << expr; \
        ::scrimmage::Logger::instance().info(_ss.str(), __FILE__, __LINE__); \
    } while (0)

#define LOG_WARN_1(expr) \
    do { \
        std::ostringstream _ss; \
        _ss << expr; \
        ::scrimmage::Logger::instance().warn(_ss.str(), __FILE__, __LINE__); \
    } while (0)

#define LOG_WARN_2(expr, ex) \
    do { \
        std::ostringstream _ss; \
        _ss << expr; \
        ::scrimmage::Logger::instance().warn(_ss.str(), ex, __FILE__, __LINE__); \
    } while (0)

#define LOG_ERROR_1(expr) \
    do { \
        std::ostringstream _ss; \
        _ss << expr; \
        ::scrimmage::Logger::instance().error(_ss.str(), __FILE__, __LINE__); \
    } while (0)

#define LOG_ERROR_2(expr, ex) \
    do { \
        std::ostringstream _ss; \
        _ss << expr; \
        ::scrimmage::Logger::instance().error(_ss.str(), ex, __FILE__, __LINE__); \
    } while (0)

#define _SC_GET_MACRO(_1, _2, NAME, ...) NAME

#define LOG_WARN(...) \
    _SC_GET_MACRO(__VA_ARGS__, LOG_WARN_2, LOG_WARN_1)(__VA_ARGS__)

#define LOG_ERROR(...) \
    _SC_GET_MACRO(__VA_ARGS__, LOG_ERROR_2, LOG_ERROR_1)(__VA_ARGS__)

#endif  // INCLUDE_SCRIMMAGE_LOG_LOGGER_H_
