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

#include <iostream>

#if ENABLE_GRPC
#include <grpc++/grpc++.h>
#include <scrimmage/proto/Scrimmage.grpc.pb.h>
#include <scrimmage/network/ScrimmageServiceImpl.h>
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
#endif

#include <scrimmage/network/Interface.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/log/Log.h>

#include <scrimmage/proto/ProtoConversions.h>

#include <thread>

using std::cout;
using std::endl;

namespace scrimmage {

    Interface::Interface() : mode_(shared), max_queue_size_(100),
                             client_timeout_(1), caching_enabled_(true)
    {
    }

    bool Interface::init_network(Interface::Mode_t mode, std::string ip,
                                 int port)
    {
        mode_ = mode;
        ip_ = ip;
        port_ = port;

        std::string result = ip_ + ":" + std::to_string(port_);

        if (mode_ == server) {
#if ENABLE_GRPC
            ScrimmageServiceImpl frame_service(this);
            grpc::ServerBuilder builder;
            builder.AddListeningPort(result, grpc::InsecureServerCredentials());
            builder.RegisterService(&frame_service);
            std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
            std::cout << "Server listening on: " << result << std::endl;
            server->Wait(); // this function blocks (should be in thread now)
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        } else if (mode_ == client) {
#if ENABLE_GRPC
            std::shared_ptr<Channel> channel(grpc::CreateChannel(result,
                                                                 grpc::InsecureChannelCredentials()));
            std::unique_ptr<scrimmage_proto::ScrimmageService::Stub> frame_temp(scrimmage_proto::ScrimmageService::NewStub(channel));
            scrimmage_stub_ = std::move(frame_temp);
            cout << "Client - Creating channel: " << result << endl;
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }

    bool Interface::send_frame(std::shared_ptr<scrimmage_proto::Frame> &frame)
    {
        log_->save_frame(frame);

        if (mode_ == shared) {
            push_frame(frame);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;

            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendFrame(&context, *frame, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_frame: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }

    bool Interface::send_frame(double time,
                               std::shared_ptr<ContactMap> &contacts)
    {
        std::shared_ptr<scrimmage_proto::Frame> frame =          \
            create_frame(time, contacts);

        return send_frame(frame);
    }

    bool Interface::send_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain)
    {
        log_->save_utm_terrain(utm_terrain);

        if (caching_enabled_) {
            utm_terrain_cache_ = utm_terrain;
        }
        
        if (mode_ == shared) {
            push_utm_terrain(utm_terrain);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;
            
            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendUTMTerrain(&context, *utm_terrain, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_utm_terrain: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }

    bool Interface::send_contact_visual(std::shared_ptr<scrimmage_proto::ContactVisual> &cv)
    {
        log_->save_contact_visual(cv);

        if (caching_enabled_) {
            contact_visual_cache_.push_back(cv);
        }
        
        if (mode_ == shared) {
            push_contact_visual(cv);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;

            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendContactVisual(&context, *cv, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_contact_visual: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }    

    bool Interface::send_gui_msg(scrimmage_proto::GUIMsg &gui_msg)
    {
        if (mode_ == shared) {
            push_gui_msg(gui_msg);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;

            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendGUIMsg(&context, gui_msg, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_gui_msg: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }    

    bool Interface::send_sim_info(scrimmage_proto::SimInfo &sim_info)
    {
        if (mode_ == shared) {
            push_sim_info(sim_info);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;

            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendSimInfo(&context, sim_info, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_sim_info: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }    

    bool Interface::send_shapes(scrimmage_proto::Shapes &shapes)
    {
        log_->save_shapes(shapes);

        if (mode_ == shared) {
            push_shapes(shapes);
        } else if (mode_ == client) {
#if ENABLE_GRPC
            scrimmage_proto::BlankReply reply;

            // Context for the client. It could be used to convey extra information to
            // the server and/or tweak certain RPC behaviors.
            grpc::ClientContext context;

            // Set timeout for API
            std::chrono::system_clock::time_point deadline =
                std::chrono::system_clock::now() + std::chrono::seconds(client_timeout_);
            context.set_deadline(deadline);

            grpc::Status status;
            status = scrimmage_stub_->SendShapes(&context, shapes, &reply);

            if (status.ok()) {
                return true;
            } else {
                cout << "send_shapes: Error code: " << status.error_code() << endl;
                cout << status.error_message() << endl;
                return false;
            }
#else
            cout << "WARNING: GRPC DISABLED!" << endl;
#endif
        }
        return true;
    }    

    void Interface::send_cached()
    {
        // Disable caching during retransmission, so we don't double save
        // cached data
        caching_enabled_ = false;
        
        // Resend terrain data...
        send_utm_terrain(utm_terrain_cache_);        

        // Resend contact visual data...
        for (auto cv : contact_visual_cache_) {
            send_contact_visual(cv);
        }
        
        // Back to normal caching mode
        caching_enabled_ = true;
    }

    bool Interface::push_contact_visual(std::shared_ptr<scrimmage_proto::ContactVisual> &cv)
    {
        contact_visual_mutex.lock();
        contact_visual_list_.push_back(cv);
        contact_visual_mutex.unlock();
        return true;
    }

    bool Interface::push_frame(std::shared_ptr<scrimmage_proto::Frame> &frame)
    {
        frames_mutex.lock();
        frames_list_.push_back(frame);
        if (frames_list_.size() > max_queue_size_) {
            frames_list_.pop_front();        
        }
        frames_mutex.unlock();
        return true;
    }

    bool Interface::push_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain)
    {
        utm_terrain_mutex.lock();
        utm_terrain_list_.push_back(utm_terrain);
        utm_terrain_mutex.unlock();
        return true;
    }

    bool Interface::push_gui_msg(scrimmage_proto::GUIMsg &gui_msg)
    {
        gui_msg_mutex.lock();
        gui_msg_list_.push_back(gui_msg);
        gui_msg_mutex.unlock();
        return true;
    }

    bool Interface::push_sim_info(scrimmage_proto::SimInfo &sim_info)
    {
        sim_info_mutex.lock();
        sim_info_list_.push_back(sim_info);
        if (sim_info_list_.size() > max_queue_size_) {
            sim_info_list_.pop_front();        
        }
        sim_info_mutex.unlock();
        return true;
    }

    bool Interface::push_shapes(scrimmage_proto::Shapes &shapes)
    {
        shapes_mutex.lock();
        shapes_list_.push_back(shapes);
        
        if (shapes_list_.size() > max_queue_size_) {
            shapes_list_.pop_front();
            // cout << "pop shapes" << endl;
        }
        shapes_mutex.unlock();
        return true;
    }
}
