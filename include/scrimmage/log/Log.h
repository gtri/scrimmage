/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_LOG_LOG_H_
#define INCLUDE_SCRIMMAGE_LOG_LOG_H_

#include <scrimmage/log/Frame.h>

#include <list>
#include <fstream>
#include <map>
#include <string>
#include <memory>

namespace google { namespace protobuf {
class MessageLite;
namespace io {
class ZeroCopyInputStream;
class ZeroCopyOutputStream;
}}}

namespace scrimmage_proto {
class Frame;
class Shapes;
class UTMTerrain;
class ContactVisual;
}

namespace scrimmage {

class Log {
 public:
    using ZeroCopyInputStreamPtr = std::shared_ptr<google::protobuf::io::ZeroCopyInputStream>;
    using ZeroCopyOutputStreamPtr = std::shared_ptr<google::protobuf::io::ZeroCopyOutputStream>;

    Log();

    enum FileType {
        FRAMES = 0,
        SHAPES = 1,
        UTMTERRAIN = 2,
        SIMINFO = 3,
        CONTACTVISUAL = 4,
        GUIMSG = 5,
        MSG = 6
    };

    enum Mode {
        READ = 0,
        WRITE = 1,
        NONE
    };

    bool init(const std::string &dir, Mode mode);

    bool parse(std::string dir);
    bool parse(std::string filename, FileType type);

    bool parse_frames(std::string filename,
                      ZeroCopyInputStreamPtr input);

    bool parse_shapes(std::string filename,
                      ZeroCopyInputStreamPtr input);

    bool parse_utm_terrain(std::string filename,
                           ZeroCopyInputStreamPtr input);

    bool parse_contact_visual(std::string filename,
                              ZeroCopyInputStreamPtr input);

    bool close_log();

    std::list<Frame> & scrimmage_frames();

    std::list<std::shared_ptr<scrimmage_proto::Frame>> & frames();
    std::list<std::shared_ptr<scrimmage_proto::Shapes>> & shapes();
    std::list<std::shared_ptr<scrimmage_proto::UTMTerrain>> & utm_terrain();
    std::list<std::shared_ptr<scrimmage_proto::ContactVisual>> & contact_visual();

    std::string log_dir();

    bool write_ascii(const std::string &str);

    bool save_frame(const std::shared_ptr<scrimmage_proto::Frame> &frame);

    bool save_shapes(const scrimmage_proto::Shapes &shapes);

    bool save_utm_terrain(const std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain);

    bool save_contact_visual(const std::shared_ptr<scrimmage_proto::ContactVisual> &contact_visual);

    // bool save_messages();

    std::string frames_filename();
    std::string shapes_filename();
    std::string utm_terrain_filename();
    std::string contact_visual_filename();
    std::string msgs_filename();

    void set_enable_log(bool enable);

    void init_network(NetworkPtr network);

 protected:
    using MessageLitePtr = std::shared_ptr<google::protobuf::MessageLite>;

    bool enable_log_ = true;
    Mode mode_ = Mode::READ;

    bool open_file(std::string name, int &fd);

    std::string frames_name_ = "frames.bin";
    std::string shapes_name_ = "shapes.bin";
    std::string utm_terrain_name_ = "utm_terrain.bin";
    std::string contact_visual_name_ = "contact_visual.bin";
    std::string msgs_name_ = "msgs.bin";

    int frames_fd_;
    int shapes_fd_;
    int utm_terrain_fd_;
    int contact_visual_fd_;
    int msgs_fd_;

    std::string log_dir_;

    std::string ascii_filename_ = "log.txt";
    std::ofstream ascii_output_;

    ZeroCopyOutputStreamPtr frames_output_;
    ZeroCopyOutputStreamPtr shapes_output_;
    ZeroCopyOutputStreamPtr utm_terrain_output_;
    ZeroCopyOutputStreamPtr contact_visual_output_;
    ZeroCopyOutputStreamPtr msgs_output_;

    std::list<Frame> scrimmage_frames_;
    std::list<std::shared_ptr<scrimmage_proto::Frame> > frames_;
    std::list<std::shared_ptr<scrimmage_proto::Shapes> > shapes_;
    std::list<std::shared_ptr<scrimmage_proto::UTMTerrain> > utm_terrain_;
    std::list<std::shared_ptr<scrimmage_proto::ContactVisual> > contact_visual_;

    bool writeDelimitedTo(
        const google::protobuf::MessageLite& message,
        ZeroCopyOutputStreamPtr rawOutput);

    bool readDelimitedFrom(const std::string &filename,
                           ZeroCopyInputStreamPtr rawInput,
                           MessageLitePtr message,
                           bool& clean_eof);
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_LOG_LOG_H_
