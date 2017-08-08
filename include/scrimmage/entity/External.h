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
#include <map>
#include <string>

#ifdef ROSCPP_ROS_H
#include <scrimmage/entity/Entity.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/autonomy/Autonomy.h>

#include <functional>
#include <vector>
#endif

namespace scrimmage {

class External {
 public:
    External();
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

#ifdef ROSCPP_ROS_H

 protected:
    std::vector<ros::Subscriber> ros_subs_;
    std::vector<ros::ServiceServer> ros_service_servers_;
    std::vector<std::function<void()>> ros_pub_funcs_;

 public:
    void publish_all() {
        for (auto &func : ros_pub_funcs_) {
            func();
        }
    }

    template <typename RosType, typename Ros2ScFunc>
    void add_subscriber(
        ros::NodeHandle &nh,
        scrimmage::SubscriberPtr sc_sub,
        Ros2ScFunc ros2sc_func,
        std::string ros_topic = "",
        std::function<void(const boost::shared_ptr<RosType const> &)> pre_func =
            [](const boost::shared_ptr<RosType const> &msg) {return;},
        std::function<void(const boost::shared_ptr<RosType const> &)> post_func =
            [](const boost::shared_ptr<RosType const> &msg) {return;}) {

        boost::function<void(const boost::shared_ptr<RosType const> &)> callback =
            [=](const boost::shared_ptr<RosType const> &ros_msg) {
                pre_func(ros_msg);
                sc_sub->msg_list().push_back(ros2sc_func(ros_msg));
                post_func(ros_msg);
            };

        std::string topic = ros_topic == "" ? sc_sub->get_topic() : ros_topic;
        ros_subs_.push_back(nh.subscribe(topic, 5, callback));
    }

    template <class ScrimmageType, class Sc2RosFunc>
    void add_publisher(
        const ros::Publisher &ros_pub,
        scrimmage::PublisherPtr sc_pub,
        Sc2RosFunc sc2ros_func,
        std::function<void()> pre_func = [](){return;},
        std::function<void()> post_func = [](){return;}) {

        auto ros_pub_ptr = std::make_shared<ros::Publisher>(ros_pub);
        auto func = [=]() {
            pre_func();
            for (auto msg : sc_pub->msgs<scrimmage::Message<ScrimmageType>>(true, false)) {
                ros_pub_ptr->publish(sc2ros_func(msg));
            }
            post_func();
        };

        ros_pub_funcs_.push_back(func);
    }

    template <class RosType, class ScrimmageResponseType, class ScServiceFunc, class Ros2ScRequestFunc, class Sc2RosResponseFunc>
    void advertise_service(
        ros::NodeHandle &nh,
        const std::string &service_name,
        Ros2ScRequestFunc ros2sc_request_func,
        ScServiceFunc sc_service_func,
        Sc2RosResponseFunc sc2ros_response_func,
        std::function<bool(typename RosType::Request&)> pre_func = [](typename RosType::Request&){return true;},
        std::function<bool(typename RosType::Response&)> post_func = [](typename RosType::Response&){return true;}) {

        boost::function<bool(typename RosType::Request &, typename RosType::Response &)> callback =
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {
                if (!pre_func(ros_req)) {
                    return false;
                }

                auto sc_req = ros2sc_request_func(ros_req);

                auto sc_req_base = std::dynamic_pointer_cast<scrimmage::MessageBase>(sc_req);
                auto sc_res_base = std::make_shared<scrimmage::MessageBase>();
                if (!sc_req_base || !sc_service_func(sc_req_base, sc_res_base)) {
                    return false;
                }

                auto sc_res = std::dynamic_pointer_cast<ScrimmageResponseType>(sc_res_base);
                if (!sc_res) {
                    return false;
                }

                ros_res = sc2ros_response_func(sc_res);
                if (!post_func(ros_res)) {
                    return false;
                }
                return true;
            };

        ros::ServiceServer srv = nh.advertiseService(service_name.c_str(), callback);
        ros_service_servers_.push_back(srv);
    }

    template <class RosServiceType, class ScrimmageReqType,
            class Sc2RosRequestFunc, class Ros2ScResponseFunc>
    add_service(ros::NodeHandle &nh,
        Sc2RosRequestFunc sc2ros_request_func,
        Ros2ScResponseFunc ros2sc_response_func,
        const std::string &sc_topic,
        const std::string &ros_topic = "") {

        std::string topic = ros_topic == "" ? sc_topic : ros_topic;
        auto service_client =
            std::make_shared<ros::ServiceClient>(nh.serviceClient<RosServiceType>(topic));

        auto call_service =
            [=](scrimmage::MessageBasePtr sc_req, scrimmage::MessageBasePtr &sc_res) {
                auto sc_req_cast =
                    std::dynamic_pointer_cast<ScrimmageReqType>(sc_req);
                if (!sc_req_cast) {
                    return false;
                }

                RosServiceType srv;
                srv.request = sc2ros_request_func(sc_req_cast);

                if (!service_client->call(srv)) {
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
