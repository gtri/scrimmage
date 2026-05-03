#ifndef C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAPSERVICEIMPL_H_
#define C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAPSERVICEIMPL_H_

#include <memory>

#include <grpcpp/grpcpp.h>

#include "TopicTap.grpc.pb.h"

namespace c2overlay {
namespace interaction {

class TopicTap;

class TopicTapServiceImpl final
    : public c2overlay_msgs::TopicTapService::Service {
 public:
  explicit TopicTapServiceImpl(TopicTap* parent) : parent_(parent) {}

  grpc::Status ListTopics(
      grpc::ServerContext* ctx,
      const c2overlay_msgs::ListTopicsRequest* req,
      c2overlay_msgs::TopicList* resp) override;

  grpc::Status StreamTopic(
      grpc::ServerContext* ctx,
      const c2overlay_msgs::StreamTopicRequest* req,
      grpc::ServerWriter<c2overlay_msgs::TopicMessage>* writer) override;

  grpc::Status PublishToTopic(
      grpc::ServerContext* ctx,
      const c2overlay_msgs::PublishToTopicRequest* req,
      c2overlay_msgs::PublishToTopicResponse* resp) override;

 private:
  TopicTap* parent_;  // not owned
};

}  // namespace interaction
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAPSERVICEIMPL_H_
