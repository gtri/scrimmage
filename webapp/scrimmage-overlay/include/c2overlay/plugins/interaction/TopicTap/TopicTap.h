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

class TopicTap : public scrimmage::EntityInteraction {
 public:
  TopicTap();
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

  void stop();

 private:
  using AddTapFn = std::function<void(TopicTap*, const TapSpec&)>;
  static const std::map<std::string, AddTapFn>& type_registry();

  template <class T>
  void add_tap(const TapSpec& spec);

  void enqueue(c2overlay_msgs::TopicMessage&& msg);

  void run_server();

  std::string ip_ = "0.0.0.0";
  int port_ = 60001;
  std::vector<TapSpec> taps_;
  std::thread server_thread_;
  std::atomic<bool> stopping_{false};

  // Per-(network,topic) ring buffers, capped at 1000 messages.
  struct Queue {
    std::mutex m;
    std::condition_variable cv;
    std::deque<c2overlay_msgs::TopicMessage> q;
    uint64_t dropped = 0;
  };
  std::map<std::string, std::shared_ptr<Queue>> queues_;
  std::mutex queues_mutex_;

  static std::string queue_key(const std::string& network, const std::string& topic) {
    return network + ":" + topic;
  }
};

}  // namespace interaction
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAP_H_
