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
 * @file Logger.cpp
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @version 1.0
 * ---------------------------------------------------------------------------
 * @brief Thread-safe logging implementation.
 *
 * Logging behavior:
 *   - All messages are printed to stdout.
 *   - File logging is opt-in via init_dir(). The Logger is agnostic to when
 *     this is called; currently SimControl enables it when output_required().
 *   - When file logging is enabled, messages also go to scrimmage.log with
 *     [LEVEL] file:line annotations.
 * ---------------------------------------------------------------------------
 */

#include "scrimmage/log/Logger.h"

#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace scrimmage {

Logger::~Logger() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
}


void Logger::init(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
    file_.open(path, std::ios::app);

    // Flush any pending messages that were buffered before init
    if (file_.is_open() && !pending_.empty()) {
        for (const auto& pm : pending_) {
            file_ << pm.formatted << std::endl;
        }
        file_.flush();
        pending_.clear();
    }
    dir_initialized_ = true;
}

void Logger::init_dir(const std::string& log_dir) {
    // Ensure the log directory exists
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(log_dir, ec);

    init(log_dir + "/scrimmage.log");
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
}

void Logger::info(const std::string& msg, const char* file, int line) {
    write("INFO", file, line, msg);
}

void Logger::warn(const std::string& msg, const char* file, int line) {
    write("WARN", file, line, msg);
}

void Logger::error(const std::string& msg, const char* file, int line) {
    write("ERROR", file, line, msg);
}

void Logger::error(const std::string& msg, const std::exception& ex,
                   const char* file, int line) {
    error(msg + ": " + ex.what(), file, line);
}

void Logger::warn(const std::string& msg, const std::exception& ex,
                  const char* file, int line) {
    warn(msg + ": " + ex.what(), file, line);
}

void Logger::write(const char* level, const char* file, int line,
                   const std::string& msg) {
    // Strip path to filename only
    const char* filename = std::strrchr(file, '/');
    filename = filename ? filename + 1 : file;

    // Full output for log file (with annotations)
    std::ostringstream ss;
    ss << "[" << level << "] ";
    if (filename && filename[0] != '\0') {
        ss << filename << ":" << line << " - ";
    }
    ss << msg;
    std::string file_output = ss.str();

    std::lock_guard<std::mutex> lock(mutex_);

    // ANSI color codes for console output
    const char* color_start = "";
    const char* color_end = "\033[0m";
    if (std::strcmp(level, "ERROR") == 0) {
        color_start = "\033[31m";  // Red
    } else if (std::strcmp(level, "WARN") == 0) {
        color_start = "\033[33m";  // Yellow
    } else {
        color_end = "";  // No reset needed if no color
    }

    // Console: plain message with color only
    std::cout << color_start << msg << color_end << std::endl;

    // If not initialized yet, buffer the message for later
    if (!dir_initialized_) {
        PendingMessage pm;
        pm.level = level;
        pm.file = file;
        pm.line = line;
        pm.msg = msg;
        pm.formatted = file_output;
        pending_.push_back(std::move(pm));
    } else if (file_.is_open()) {
        file_ << file_output << std::endl;  // Full annotations in file
    }

    // Flush immediately on ERROR to avoid losing logs on crash
    if (std::strcmp(level, "ERROR") == 0) {
        std::cout.flush();
        if (dir_initialized_ && file_.is_open()) file_.flush();
    }
}

}  // namespace scrimmage
