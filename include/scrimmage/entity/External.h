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
#ifndef INCLUDE_SCRIMMAGE_ENTITY_EXTERNAL_H_
#define INCLUDE_SCRIMMAGE_ENTITY_EXTERNAL_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include <map>
#include <string>

#ifdef ROSCPP_ROS_H
#include <scrimmage/entity/Entity.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/autonomy/Autonomy.h>

#include <iostream>
#include <functional>
#include <vector>

#include <boost/type_index.hpp>
#endif

namespace scrimmage {

#ifdef ROSCPP_ROS_H
template <class RosType, class Ros2Sc, class PostFunc>
boost::function<void(const boost::shared_ptr<RosType const>&)>
create_cb_post(Ros2Sc ros2sc, SubscriberPtr sc_sub, PostFunc post_func) {
    return [=](const boost::shared_ptr<RosType const> &ros_msg) {
        sc_sub->add_msg(ros2sc(ros_msg));
        post_func();
    };
}

template <class RosType, class Ros2Sc>
boost::function<void(const boost::shared_ptr<RosType const>&)>
create_cb(Ros2Sc ros2sc, SubscriberPtr sc_sub) {
    return [=](const boost::shared_ptr<RosType const> &ros_msg) {
        sc_sub->add_msg(ros2sc(ros_msg));
    };
}

#endif

class External {
 public:
    External() :
        plugin_manager_(std::make_shared<PluginManager>()), max_entities_(-1) {}

    NetworkPtr &network();
    EntityPtr &entity();
    FileSearch &file_search();
    PluginManagerPtr &plugin_manager();
    int get_max_entities();
    void set_max_entities(int max_entities);
    bool create_entity(ID id,
            std::map<std::string, std::string> &info,
            const std::string &log_dir);

 protected:
    NetworkPtr network_;
    EntityPtr entity_;
    FileSearch file_search_;
    PluginManagerPtr plugin_manager_;
    int max_entities_;
    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

#ifdef ROSCPP_ROS_H

 protected:
    std::vector<ros::ServiceServer> ros_service_servers_;
    std::vector<std::function<void()>> ros_pub_funcs_;

 public:
    void publish_all() {
        for (auto &func : ros_pub_funcs_) {
            func();
        }
    }

    template <class ScrimmageType, class Sc2RosFunc>
    void add_pub(
        const ros::Publisher &ros_pub,
        scrimmage::PublisherPtr sc_pub,
        Sc2RosFunc sc2ros_func,
        std::function<void()> pre_func = nullptr,
        std::function<void()> post_func = nullptr) {

        auto ros_pub_ptr = std::make_shared<ros::Publisher>(ros_pub);
        auto func = [=]() {
            if (pre_func) pre_func();
            for (auto msg : sc_pub->msgs<scrimmage::Message<ScrimmageType>>(true, false)) {
                ros_pub_ptr->publish(sc2ros_func(msg));
            }
            if (post_func) post_func();
        };

        ros_pub_funcs_.push_back(func);
    }

    template <class RosType, class ScrimmageResponseType, class Sc2RosResponseFunc>
    void add_srv_server(
        ros::NodeHandle &nh,
        const std::string &service_name,
        scrimmage::Service sc_service_func,
        Sc2RosResponseFunc sc2ros_response_func,
        std::function<bool(typename RosType::Request&)> pre_func = nullptr,
        std::function<bool(typename RosType::Response&)> post_func = nullptr) {

        boost::function<bool(typename RosType::Request &, typename RosType::Response &)> callback =
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {

                std::string suffix =
                    std::string(" in advertised_service \"") + service_name + "\"";

                if (pre_func && !pre_func(ros_req)) {
                    std::cout << "call to pre_func converting ros to scrimmage request failed"
                        << suffix << std::endl;
                    return false;
                }

                auto sc_req = std::shared_ptr<MessageBase>();

                auto sc_res_base = std::make_shared<scrimmage::MessageBase>();
                if (!sc_service_func(sc_req, sc_res_base)) {
                    std::cout << "call to sc_service_func failed" << suffix << std::endl;
                    return false;
                }

                auto sc_res =
                    std::dynamic_pointer_cast<scrimmage::Message<ScrimmageResponseType>>(sc_res_base);
                if (!sc_res) {
                    std::cout << "could not cast to "
                        << "scrimmage::MessagePtr<ScrimmageResponseType> "
                        << "(aka, scrimmage::MessagePtr<"
                        << boost::typeindex::type_id<ScrimmageResponseType>().pretty_name()
                        << ">)" << suffix << std::endl;
                    return false;
                }

                ros_res = sc2ros_response_func(sc_res);
                if (post_func && !post_func(ros_res)) {
                    std::cout << "call to post_func converting scrimmage to "
                        << "ros response failed" << suffix << std::endl;
                    return false;
                }
                return true;
            };

        ros_service_servers_.push_back(nh.advertiseService(service_name.c_str(), callback));
    }

    template <class RosType, class ScrimmageResponseType,
              class Ros2ScRequestFunc, class Sc2RosResponseFunc>
    void add_srv_server(
        ros::NodeHandle &nh,
        const std::string &service_name,
        scrimmage::Service sc_service_func,
        Ros2ScRequestFunc ros2sc_request_func,
        Sc2RosResponseFunc sc2ros_response_func,
        std::function<bool(typename RosType::Request&)> pre_func = nullptr,
        std::function<bool(typename RosType::Response&)> post_func = nullptr) {

        boost::function<bool(typename RosType::Request &, typename RosType::Response &)> callback =
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {

                auto err_msg = [&](const std::string &preface) {
                    std::cout << preface << " in advertised_service \""
                        << service_name << "\"" << std::endl;
                };

                if (pre_func && !pre_func(ros_req)) {
                    err_msg("call to pre_func converting ros to scrimmage request failed");
                    return false;
                }

                auto sc_req = ros2sc_request_func(ros_req);

                auto sc_req_base = std::dynamic_pointer_cast<scrimmage::MessageBase>(sc_req);
                if (sc_req_base == nullptr) {
                    err_msg("could not cast scrimmage request scrimmage::MessageBasePtr");
                    return false;
                }

                auto sc_res_base = std::make_shared<scrimmage::MessageBase>();
                if (!sc_service_func(sc_req_base, sc_res_base)) {
                    err_msg("call to sc_service_func failed");
                    return false;
                }

                auto sc_res =
                    std::dynamic_pointer_cast<scrimmage::Message<ScrimmageResponseType>>(sc_res_base);
                if (!sc_res) {
                    std::stringstream ss;
                    ss << "could not cast to scrimmage::MessagePtr<ScrimmageResponseType> "
                        << "(aka, scrimmage::MessagePtr<"
                        << boost::typeindex::type_id<ScrimmageResponseType>().pretty_name()
                        << ">)";
                    err_msg(ss.str());
                    return false;
                }

                ros_res = sc2ros_response_func(sc_res);
                if (post_func && !post_func(ros_res)) {
                    err_msg("call to post_func converting scrimmage to ros response failed");
                    return false;
                }
                return true;
            };

        ros_service_servers_.push_back(nh.advertiseService(service_name.c_str(), callback));
    }

    template <class RosType, class Ros2ScResponseFunc>
    void add_srv_client(ros::NodeHandle &nh,
        const std::string &sc_topic,
        Ros2ScResponseFunc ros2sc_response_func,
        const std::string &ros_topic = "") {

        std::string topic = ros_topic == "" ? sc_topic : ros_topic;
        auto service_client =
            std::make_shared<ros::ServiceClient>(nh.serviceClient<RosType>(topic));

        auto call_service =
            [=](scrimmage::MessageBasePtr sc_req, scrimmage::MessageBasePtr &sc_res) {

                RosType srv;
                if (!service_client->call(srv)) {
                    std::cout << "service call failed "
                        << "(sc_topic = " + sc_topic + ", ros_topic = " + topic + ")"
                        << std::endl;
                    return false;
                }

                sc_res = ros2sc_response_func(srv.response);
                return true;
            };

        entity_->services()[sc_topic] = call_service;
    }

    template <class RosType, class ScrimmageRequestType,
              class Sc2RosRequestFunc, class Ros2ScResponseFunc>
    void add_srv_client(ros::NodeHandle &nh,
        const std::string &sc_topic,
        Sc2RosRequestFunc sc2ros_request_func,
        Ros2ScResponseFunc ros2sc_response_func,
        const std::string &ros_topic = "") {

        std::string topic = ros_topic == "" ? sc_topic : ros_topic;
        auto service_client =
            std::make_shared<ros::ServiceClient>(nh.serviceClient<RosType>(topic));

        auto call_service =
            [=](scrimmage::MessageBasePtr sc_req, scrimmage::MessageBasePtr &sc_res) {
                std::string suffix =
                    std::string("(sc_topic = ") + sc_topic + ", ros_topic = " + topic + ")";

                auto sc_req_cast =
                    std::dynamic_pointer_cast<scrimmage::Message<ScrimmageRequestType>>(sc_req);
                if (!sc_req_cast) {
                    std::cout << "could not cast scrimmage::MessageBase request "
                        << " to scrimmage::MessagePtr<"
                        << boost::typeindex::type_id<ScrimmageRequestType>().pretty_name()
                        << "> " << suffix << std::endl;
                    return false;
                }

                RosType srv;
                srv.request = sc2ros_request_func(sc_req_cast);

                if (!service_client->call(srv)) {
                    std::cout << "service call failed " << suffix << std::endl;
                    return false;
                }

                sc_res = ros2sc_response_func(srv.response);
                return true;
            };

        entity_->services()[sc_topic] = call_service;
    }
#endif
};

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_ENTITY_EXTERNAL_H_
