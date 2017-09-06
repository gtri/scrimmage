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

#include <iostream>
#include <functional>
#include <vector>

#include <boost/type_index.hpp>
#endif

namespace scrimmage {

#ifdef ROSCPP_ROS_H
template <class RosType>
scrimmage::MessagePtr<RosType> ros2sc_msg(const boost::shared_ptr<RosType const>& ros_msg) {
    return std::make_shared<scrimmage::Message<RosType>>(*ros_msg);
}

template <class RosType>
RosType sc2ros_msg(const scrimmage::MessagePtr<RosType> &sc_msg) {
    return sc_msg->data;
}

#endif

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

    template <typename RosType, typename ScrimmageType>
    void add_subscriber(
        ros::NodeHandle &nh,
        scrimmage::SubscriberPtr sc_sub,
        std::function<scrimmage::MessagePtr<ScrimmageType>(const boost::shared_ptr<RosType const>&)> ros2sc_func,
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

    template <typename RosType>
    void add_subscriber(
        ros::NodeHandle &nh,
        scrimmage::SubscriberPtr sc_sub,
        std::string ros_topic = "",
        std::function<scrimmage::MessagePtr<RosType>(const boost::shared_ptr<RosType const>&)>
            ros2sc_func = ros2sc_msg<RosType>,
        std::function<void(const boost::shared_ptr<RosType const> &)> pre_func =
            [](const boost::shared_ptr<RosType const> &msg) {return;},
        std::function<void(const boost::shared_ptr<RosType const> &)> post_func =
            [](const boost::shared_ptr<RosType const> &msg) {return;}) {

        add_subscriber<RosType, RosType>(nh, sc_sub, ros_topic, ros2sc_func, pre_func, post_func);
    }

    template <class RosType, class ScrimmageType>
    void add_publisher(
        const ros::Publisher &ros_pub,
        scrimmage::PublisherPtr sc_pub,
        std::function<boost::shared_ptr<RosType>(const scrimmage::MessagePtr<ScrimmageType>&)> sc2ros_func,
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

    template <class RosType>
    void add_publisher(
        const ros::Publisher &ros_pub,
        scrimmage::PublisherPtr sc_pub,
        std::function<boost::shared_ptr<RosType>(const scrimmage::MessagePtr<RosType>&)>
            sc2ros_func = ros2sc_msg<RosType>,
        std::function<void()> pre_func = [](){return;},
        std::function<void()> post_func = [](){return;}) {

        add_publisher<RosType, RosType>(ros_pub, sc_pub, sc2ros_func, pre_func, post_func);
    }

    template <class RosType, class ScrimmageRequestType, class ScrimmageResponseType>
    void advertise_service(
        ros::NodeHandle &nh,
        const std::string &service_name,
        std::function<bool(scrimmage::MessageBasePtr, scrimmage::MessageBasePtr&)> sc_service_func,
        std::function<scrimmage::MessagePtr<ScrimmageRequestType>(typename RosType::Request &)> ros2sc_request_func,
        std::function<typename RosType::Response(scrimmage::MessagePtr<ScrimmageResponseType>&)> sc2ros_response_func,
        std::function<bool(typename RosType::Request&)> pre_func = [](typename RosType::Request&){return true;},
        std::function<bool(typename RosType::Response&)> post_func = [](typename RosType::Response&){return true;}) {

        boost::function<bool(typename RosType::Request &, typename RosType::Response &)> callback =
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {

                auto err_msg = [&](const std::string &preface) {
                    std::cout << preface << " in advertised_service \""
                        << service_name << "\"" << std::endl;
                };

                if (!pre_func(ros_req)) {
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
                if (!post_func(ros_res)) {
                    err_msg("call to post_func converting scrimmage to ros response failed");
                    return false;
                }
                return true;
            };

        ros::ServiceServer srv = nh.advertiseService(service_name.c_str(), callback);
        ros_service_servers_.push_back(srv);
    }

    template <class RosType>
    void advertise_service(
        ros::NodeHandle &nh,
        const std::string &service_name,
        std::function<bool(scrimmage::MessageBasePtr, scrimmage::MessageBasePtr&)> sc_service_func,
        std::function<scrimmage::MessagePtr<typename RosType::Request>(typename RosType::Request &)>
            ros2sc_request_func = ros2sc_msg<RosType>,
        std::function<typename RosType::Response(scrimmage::MessagePtr<typename RosType::Response>&)>
            sc2ros_response_func = sc2ros_msg<RosType>,
        std::function<bool(typename RosType::Request&)> pre_func = [](typename RosType::Request&){return true;},
        std::function<bool(typename RosType::Response&)> post_func = [](typename RosType::Response&){return true;}) {

        advertise_service<RosType, RosType>(
            nh, service_name, sc_service_func, ros2sc_request_func,
            sc2ros_response_func, pre_func, post_func);
    }

    template <class RosServiceType, class ScrimmageRequestType>
    void add_service(ros::NodeHandle &nh,
        const std::string &sc_topic,
        std::function<typename RosServiceType::Request(scrimmage::MessagePtr<ScrimmageRequestType>&)>
            sc2ros_request_func,
        std::function<scrimmage::MessageBasePtr(typename RosServiceType::Response&)>
            ros2sc_response_func,
        const std::string &ros_topic = "") {

        std::string topic = ros_topic == "" ? sc_topic : ros_topic;
        auto service_client =
            std::make_shared<ros::ServiceClient>(nh.serviceClient<RosServiceType>(topic));

        auto call_service =
            [=](scrimmage::MessageBasePtr sc_req, scrimmage::MessageBasePtr &sc_res) {
                auto err_msg = [&](const std::string &preface) {
                    std::cout << preface << " in call_service "
                        << "(sc_topic = " << sc_topic << ", ros_topic = " << topic << ")"
                        << std::endl;
                };

                auto sc_req_cast =
                    std::dynamic_pointer_cast<scrimmage::Message<ScrimmageRequestType>>(sc_req);
                if (!sc_req_cast) {
                    std::stringstream ss;
                    ss << "could not cast scrimmage::MessageBase request to scrimmage::MessagePtr<"
                        << boost::typeindex::type_id<ScrimmageRequestType>().pretty_name()
                        << ">)";
                    err_msg(ss.str());
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

        std::cout << "adding call_service" << std::endl;
        entity_->services()[sc_topic] = call_service;
    }

    template <class RosType>
    void add_service(ros::NodeHandle &nh,
        const std::string &sc_topic,
        std::function<typename RosType::Request(scrimmage::MessagePtr<typename RosType::Request>&)>
            sc2ros_request_func,
        std::function<scrimmage::MessagePtr<typename RosType::Response>(typename RosType::Response&)>
            ros2sc_response_func = ros2sc_msg<typename RosType::Response>,
        const std::string &ros_topic = "") {

        add_service<RosType, RosType>(nh, sc2ros_request_func, ros2sc_response_func, sc_topic, ros_topic);
    }

#endif
};

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_ENTITY_EXTERNAL_H_
