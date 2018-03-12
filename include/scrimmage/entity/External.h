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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/math/State.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>

#include <list>
#include <map>
#include <string>
#include <functional>
#include <mutex> // NOLINT

#include <boost/type_index.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace scrimmage {

class ID;

class External {
 public:
    External();
    EntityPtr &entity();
    bool create_entity(const std::string &mission_file,
                       int max_entities, int entity_id,
                       const std::string &entity_name);

    std::function<void()> update_contacts;
    double min_motion_dt = 1;

 protected:
    EntityPtr entity_;
    PluginManagerPtr plugin_manager_;
    std::shared_ptr<Log> log_;
    double last_t_;
    std::mutex mutex_;
    PubSubPtr pubsub_;
    TimePtr time_;
    MissionParsePtr mp_;

 public:
    bool step(double t);
    PubSubPtr pubsub() {
        return pubsub_;
    }

#ifdef ROSCPP_ROS_H

 public:
    template <class Sc2Ros>
        void pub_cb(std::string network_name, std::string topic_name,
                    Sc2Ros sc2ros, ros::Publisher ros_pub) {
        auto ros_pub_ptr = std::make_shared<ros::Publisher>(ros_pub);

        boost::optional<std::list<NetworkDevicePtr>> pubs =
            pubsub_->find_pubs(network_name, topic_name);

        if (pubs) {
            for (NetworkDevicePtr dev : *pubs) {
                PublisherPtr pub = std::dynamic_pointer_cast<Publisher>(dev);
                pub->callback = [=](MessageBasePtr sc_msg) {
                    ros_pub_ptr->publish(sc2ros(sc_msg));
                };
            }
        } else {
            std::cout << "Failed to setup scrimmage to ROS publisher." << std::endl;
        }
    }

    template <class ScType, class Sc2Ros>
    void pub_cb(std::string network_name, std::string topic_name,
                Sc2Ros sc2ros, ros::Publisher ros_pub) {

        auto ros_pub_ptr = std::make_shared<ros::Publisher>(ros_pub);

        boost::optional<std::list<NetworkDevicePtr>> pubs =
            pubsub_->find_pubs(network_name, topic_name);

        if (pubs) {
            for (NetworkDevicePtr dev : *pubs) {
                PublisherPtr pub = std::dynamic_pointer_cast<Publisher>(dev);

                pub->callback = [=](MessageBasePtr sc_msg) {
                    auto sc_msg_cast = std::dynamic_pointer_cast<Message<ScType>>(sc_msg);
                    if (sc_msg_cast == nullptr) {
                        std::cout << "could not cast to "
                                  << boost::typeindex::type_id<Message<ScType>>().pretty_name()
                                  << " in pub_cb" << std::endl;
                    } else {
                        ros_pub_ptr->publish(sc2ros(sc_msg_cast));
                    }
                };
            }
        } else {
            std::cout << "Failed to setup scrimmage to ROS publisher." << std::endl;
        }
    }

    template <class RosType, class Ros2Sc>
    boost::function<void(const boost::shared_ptr<RosType const>&)>
    sub_cb(std::string network_name, std::string topic_name, Ros2Sc ros2sc) {

        boost::optional<std::list<NetworkDevicePtr>> subs =
            pubsub_->find_subs(network_name, topic_name);

        if (subs) {
            for (NetworkDevicePtr dev : *subs) {
                return [=](const boost::shared_ptr<RosType const>&ros_msg) {
                    auto sc_msg = ros2sc(*ros_msg);
                    auto sc_msg_ptr = std::make_shared<decltype(sc_msg)>();
                    *sc_msg_ptr = sc_msg;
                    dev->add_msg(sc_msg_ptr);
                    // step(ros::Time::now().toSec());
                };
            }
        }

        std::cout << "Failed to create ROS to SCRIMMAGE subscriber (callback) " << std::endl;
        std::cout << "Network name: " << network_name << std::endl;
        std::cout << "Topic name: " << topic_name << std::endl;
        return [=](const boost::shared_ptr<RosType const>&ros_msg) { };
    }

    template <class RosType, class ScrimmageResponseType, class Sc2RosResponseFunc>
    boost::function<bool(typename RosType::Request &, typename RosType::Response &)>
    srv_cb(const std::string &service_name,
           scrimmage::Service sc_service_func,
           Sc2RosResponseFunc sc2ros_response_func) {

        return
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {

                if (update_contacts) {
                    mutex_.lock();
                    update_contacts();
                    mutex_.unlock();
                }
                std::string suffix =
                    std::string(" in advertised_service \"") + service_name + "\"";

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
                return true;
            };
    }

    template <class RosType, class ScrimmageResponseType,
              class Ros2ScRequestFunc, class Sc2RosResponseFunc>
    boost::function<bool(typename RosType::Request &, typename RosType::Response &)>
    srv_cb(const std::string &service_name,
           scrimmage::Service sc_service_func,
           Ros2ScRequestFunc ros2sc_request_func,
           Sc2RosResponseFunc sc2ros_response_func) {

        return
            [=](typename RosType::Request &ros_req, typename RosType::Response &ros_res) {

                update_contacts();
                auto err_msg = [&](const std::string &preface) {
                    std::cout << preface << " in advertised_service \""
                        << service_name << "\"" << std::endl;
                };

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
                return true;
            };
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
