#ifndef C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAP_H_
#define C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAP_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "scrimmage/simcontrol/EntityInteraction.h"
#include "TopicTap.pb.h"

namespace c2overlay {
namespace interaction {

struct TapSpec {
  std::string network;
  std::string topic;
  std::string type_name;
};

// Forward-declare grpc::Server so the header doesn't pull in <grpcpp/grpcpp.h>.
}  // namespace interaction
}  // namespace c2overlay
namespace grpc { class Server; }
namespace c2overlay {
namespace interaction {

class TopicTap : public scrimmage::EntityInteraction {
 public:
  TopicTap();
  ~TopicTap() override;
  bool init(
      std::map<std::string, std::string>& mission_params,
      std::map<std::string, std::string>& plugin_params) override;
  bool step_entity_interaction(
      std::list<scrimmage::EntityPtr>& ents,
      double t,
      double dt) override;

  std::vector<TapSpec> taps() const;

  // Stream-side interface (called from gRPC service thread).
  // Blocks up to 1s waiting for a message matching (network, topic).
  // Returns false if shutting down.
  bool wait_for_message(
      const std::string& network,
      const std::string& topic,
      c2overlay_msgs::TopicMessage* out);

  // Publish-side interface (called from gRPC service thread).
  // Validates + parses payload, enqueues for sim-thread publish.
  // Returns (ok, error) — empty error on success.
  std::pair<bool, std::string> enqueue_publish(
      const std::string& network,
      const std::string& topic,
      const std::string& payload_json);

  void stop();

 private:
  using AddTapFn = std::function<void(TopicTap*, const TapSpec&)>;
  static const std::map<std::string, AddTapFn>& type_registry();

  template <class T>
  void add_tap(const TapSpec& spec);

  void enqueue(c2overlay_msgs::TopicMessage&& msg);

  void run_server();

  // Sim-thread side of publish path (called from step_entity_interaction).
  void drain_publish_queue();

  // Publish handler: parses JSON, builds typed message, returns serialized
  // form ready for the sim thread to publish. Returns false on parse error
  // (with error written to *err_out).
  using PublishHandlerFn = std::function<
      bool(const std::string& payload_json,
           std::shared_ptr<scrimmage::MessageBase>* msg_out,
           std::string* err_out)>;
  static const std::map<std::string, PublishHandlerFn>& publish_registry();

  std::string ip_ = "0.0.0.0";
  int port_ = 60001;
  std::vector<TapSpec> taps_;
  std::thread server_thread_;
  std::atomic<bool> stopping_{false};
  std::unique_ptr<grpc::Server> server_;

  // Per-(network,topic) ring buffers, capped at 1000 messages.
  struct Queue {
    std::mutex m;
    std::condition_variable cv;
    std::deque<c2overlay_msgs::TopicMessage> q;
    uint64_t dropped = 0;
  };
  std::map<std::string, std::shared_ptr<Queue>> queues_;
  std::mutex queues_mutex_;

  // Publish-path state.
  struct PendingPublish {
    std::string network;
    std::string topic;
    std::shared_ptr<scrimmage::MessageBase> msg;
  };
  std::deque<PendingPublish> pending_publishes_;
  std::mutex pending_publishes_mutex_;
  // Publisher cache populated lazily on the sim thread; no lock needed.
  std::map<std::string, scrimmage::PublisherPtr> pub_cache_;

  static std::string queue_key(const std::string& network, const std::string& topic) {
    return network + ":" + topic;
  }
};

}  // namespace interaction
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAP_H_
