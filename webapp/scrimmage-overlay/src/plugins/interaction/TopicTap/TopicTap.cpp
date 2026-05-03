#include "c2overlay/plugins/interaction/TopicTap/TopicTap.h"

#include <chrono>
#include <iostream>
#include <utility>

#include <google/protobuf/util/json_util.h>
#include <grpcpp/grpcpp.h>

#include "scrimmage/log/Logger.h"
#include "scrimmage/msgs/Capture.pb.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/pubsub/Message.h"

#include "c2overlay/plugins/interaction/TopicTap/TopicTapServiceImpl.h"

REGISTER_PLUGIN(
    scrimmage::EntityInteraction,
    c2overlay::interaction::TopicTap,
    TopicTap_plugin)

namespace c2overlay {
namespace interaction {

constexpr size_t kQueueMaxSize = 1000;
constexpr int kDropLogEveryN = 100;

template <class T>
static std::string proto_to_json(const T& msg) {
  std::string out;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = false;  // camelCase output
  auto status = google::protobuf::util::MessageToJsonString(msg, &out, opts);
  if (!status.ok()) {
    return std::string("{\"error\":\"") + std::string(status.message()) + "\"}";
  }
  return out;
}

const std::map<std::string, TopicTap::AddTapFn>& TopicTap::type_registry() {
  static const std::map<std::string, AddTapFn> reg = {
      {"scrimmage_msgs.CaptureEntity",
       [](TopicTap* self, const TapSpec& s) {
         self->add_tap<scrimmage_msgs::CaptureEntity>(s);
       }},
      // Add new tappable types here. Each addition requires a rebuild.
  };
  return reg;
}

TopicTap::TopicTap() = default;

template <class T>
void TopicTap::add_tap(const TapSpec& spec) {
  auto cb = [this, spec](scrimmage::MessagePtr<T> msg) {
    c2overlay_msgs::TopicMessage out;
    out.set_network(spec.network);
    out.set_topic(spec.topic);
    out.set_type_name(spec.type_name);
    out.set_t_sim(msg->time);
    out.set_payload_json(proto_to_json(msg->data));
    enqueue(std::move(out));
  };
  this->subscribe<T>(spec.network, spec.topic, cb);
}

void TopicTap::enqueue(c2overlay_msgs::TopicMessage&& msg) {
  std::shared_ptr<Queue> q;
  {
    std::lock_guard<std::mutex> g(queues_mutex_);
    auto k = queue_key(msg.network(), msg.topic());
    auto it = queues_.find(k);
    if (it == queues_.end()) {
      q = std::make_shared<Queue>();
      queues_[k] = q;
    } else {
      q = it->second;
    }
  }
  std::lock_guard<std::mutex> g(q->m);
  if (q->q.size() >= kQueueMaxSize) {
    q->q.pop_front();
    q->dropped++;
    if (q->dropped % kDropLogEveryN == 1) {
      std::cerr << "[TopicTap] dropped " << q->dropped
                << " messages on " << msg.network() << ":" << msg.topic()
                << " (consumer too slow)\n";
    }
  }
  q->q.push_back(std::move(msg));
  q->cv.notify_one();
}

bool TopicTap::wait_for_message(
    const std::string& network,
    const std::string& topic,
    c2overlay_msgs::TopicMessage* out) {
  std::shared_ptr<Queue> q;
  {
    std::lock_guard<std::mutex> g(queues_mutex_);
    auto k = queue_key(network, topic);
    auto it = queues_.find(k);
    if (it == queues_.end()) {
      // Create the queue lazily so the first stream RPC for an unfired topic
      // doesn't busy-loop returning empty.
      q = std::make_shared<Queue>();
      queues_[k] = q;
    } else {
      q = it->second;
    }
  }
  std::unique_lock<std::mutex> g(q->m);
  q->cv.wait_for(g, std::chrono::seconds(1),
                 [&] { return !q->q.empty() || stopping_.load(); });
  if (stopping_.load()) return false;
  if (q->q.empty()) return true;  // timeout, no message — caller polls again
  *out = std::move(q->q.front());
  q->q.pop_front();
  return true;
}

bool TopicTap::init(
    std::map<std::string, std::string>& /*mission_params*/,
    std::map<std::string, std::string>& plugin_params) {
  ip_ = scrimmage::get<std::string>("ip", plugin_params, ip_);
  port_ = scrimmage::get<int>("port", plugin_params, port_);

  // Plugin XML child elements arrive flattened with "tap" → comma-joined values.
  // Operators specify taps via repeated <tap network="..." topic="..." type="..."/>.
  // We accept three parallel flat keys: "tap_network", "tap_topic", "tap_type".
  std::string nets = scrimmage::get<std::string>("tap_network", plugin_params, "");
  std::string tops = scrimmage::get<std::string>("tap_topic", plugin_params, "");
  std::string typs = scrimmage::get<std::string>("tap_type", plugin_params, "");

  auto split = [](const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
      if (c == ',') { out.push_back(cur); cur.clear(); }
      else { cur += c; }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
  };

  auto netv = split(nets);
  auto topv = split(tops);
  auto typv = split(typs);

  if (netv.size() != topv.size() || topv.size() != typv.size()) {
    std::cerr << "[TopicTap] tap_network/tap_topic/tap_type must have equal length\n";
    return false;
  }

  const auto& reg = type_registry();
  for (size_t i = 0; i < netv.size(); ++i) {
    TapSpec spec{netv[i], topv[i], typv[i]};
    auto it = reg.find(spec.type_name);
    if (it == reg.end()) {
      std::cerr << "[TopicTap] unknown type '" << spec.type_name
                << "', skipping\n";
      continue;
    }
    it->second(this, spec);
    taps_.push_back(spec);
    std::cout << "[TopicTap] tapped " << spec.network << ":" << spec.topic
              << " (" << spec.type_name << ")\n";
  }

  server_thread_ = std::thread(&TopicTap::run_server, this);
  return true;
}

bool TopicTap::step_entity_interaction(
    std::list<scrimmage::EntityPtr>& /*ents*/,
    double /*t*/,
    double /*dt*/) {
  return true;
}

std::vector<TapSpec> TopicTap::taps() const {
  return taps_;
}

void TopicTap::run_server() {
  std::string addr = ip_ + ":" + std::to_string(port_);
  TopicTapServiceImpl service(this);
  grpc::ServerBuilder builder;
  builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "[TopicTap] gRPC listening on " << addr << "\n";
  server->Wait();
}

void TopicTap::stop() {
  stopping_.store(true);
  // Wake all consumers
  std::lock_guard<std::mutex> g(queues_mutex_);
  for (auto& kv : queues_) kv.second->cv.notify_all();
}

}  // namespace interaction
}  // namespace c2overlay
