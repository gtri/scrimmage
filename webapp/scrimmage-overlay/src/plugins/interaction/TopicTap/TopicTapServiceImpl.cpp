#include "c2overlay/plugins/interaction/TopicTap/TopicTapServiceImpl.h"

#include "c2overlay/plugins/interaction/TopicTap/TopicTap.h"

namespace c2overlay {
namespace interaction {

grpc::Status TopicTapServiceImpl::ListTopics(
    grpc::ServerContext* /*ctx*/,
    const c2overlay_msgs::ListTopicsRequest* /*req*/,
    c2overlay_msgs::TopicList* resp) {
  for (const auto& t : parent_->taps()) {
    auto* s = resp->add_topics();
    s->set_network(t.network);
    s->set_topic(t.topic);
    s->set_type_name(t.type_name);
  }
  return grpc::Status::OK;
}

grpc::Status TopicTapServiceImpl::StreamTopic(
    grpc::ServerContext* ctx,
    const c2overlay_msgs::StreamTopicRequest* req,
    grpc::ServerWriter<c2overlay_msgs::TopicMessage>* writer) {
  while (!ctx->IsCancelled()) {
    c2overlay_msgs::TopicMessage msg;
    if (!parent_->wait_for_message(req->network(), req->topic(), &msg)) {
      break;  // shutting down
    }
    if (msg.topic().empty()) continue;  // timeout, keep polling
    if (!writer->Write(msg)) break;
  }
  return grpc::Status::OK;
}

grpc::Status TopicTapServiceImpl::PublishToTopic(
    grpc::ServerContext* /*ctx*/,
    const c2overlay_msgs::PublishToTopicRequest* req,
    c2overlay_msgs::PublishToTopicResponse* resp) {
  auto [ok, err] = parent_->enqueue_publish(
      req->network(), req->topic(), req->payload_json());
  resp->set_ok(ok);
  if (!ok) resp->set_error(err);
  return grpc::Status::OK;
}

}  // namespace interaction
}  // namespace c2overlay
