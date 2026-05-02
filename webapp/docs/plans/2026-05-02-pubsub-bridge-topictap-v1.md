# SCRIMMAGE Pub/Sub Bridge (TopicTap v1) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a SCRIMMAGE C++ plugin (`TopicTap`) that bridges in-process pub/sub topics out via gRPC, plus the API and web pieces that surface a real-time topic feed in the operator console. Demo: live "kill feed" from `predator_prey_boids.xml` showing each `CaptureEntity` event published by the `Predator` autonomy plugin.

**Architecture:** New `EntityInteraction` plugin built as a SCRIMMAGE *external project overlay* (so we don't have to rebuild the prebuilt `gtri/scrimmage-24.04` base image). Plugin runs a gRPC server on `:60001` exposing `ListTopics` and a server-streaming `StreamTopic`. API gains a paired `BackgroundService` + SignalR `TopicHub` + `GET /api/topics`. Web replaces its stub topics panel with a dropdown + scrolling feed. Observe-only in v1; inject is deferred to v2.

**Tech Stack:** C++ + CMake + protobuf + gRPC (SCRIMMAGE plugin); .NET 10 + `Grpc.Net.Client` + SignalR (API); React + TypeScript + `@microsoft/signalr` (web); Docker Compose for orchestration.

**Working directory:** All `git` commands run from `C:\Git\kinetas\scrimmage\` (the SCRIMMAGE fork). All file paths in this plan are relative to that root unless absolute.

**Spec:** `webapp/docs/specs/2026-05-02-pubsub-bridge-design.md`

---

## Pre-flight

Before starting, verify:

- [ ] MVP plan (`webapp/docs/plans/2026-05-02-scrimmage-c2-ui-mvp.md`) is implemented and `docker compose up --build` from `webapp/` succeeds end-to-end with `capture-the-flag.xml`.
- [ ] `git status` shows a clean tree (or only the design doc + this plan untracked). Branch: `ui-effort` or a child branch.
- [ ] Inside the existing `c2-scrimmage` container: `which g++ cmake protoc grpc_cpp_plugin` all resolve. (The base `gtri/scrimmage-24.04` image ships these — if any are missing, the Dockerfile in Task 7 needs an `apt-get install` line.)

```bash
docker compose -f webapp/docker-compose.yml up -d scrimmage
docker exec c2-scrimmage bash -c 'which g++ cmake protoc grpc_cpp_plugin'
docker compose -f webapp/docker-compose.yml down
```

Expected output: four absolute paths, one per binary. If `grpc_cpp_plugin` is missing, add `protobuf-compiler-grpc` to the `apt-get install` line in `webapp/scrimmage-runner/Dockerfile` (Task 7) before proceeding.

---

## Task 1: Overlay project skeleton

Create the SCRIMMAGE external-project overlay that will hold our plugin. Pattern documented in `development-docs/EXTERNAL_PROJECTS.md`. We hand-write the minimal files instead of running `scripts/create-scrimmage-project.py` so the structure is visible in git.

**Files:**
- Create: `webapp/scrimmage-overlay/CMakeLists.txt`
- Create: `webapp/scrimmage-overlay/.gitignore`
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/.gitkeep`
- Create: `webapp/scrimmage-overlay/src/plugins/.gitkeep`
- Create: `webapp/scrimmage-overlay/missions/.gitkeep`
- Create: `webapp/scrimmage-overlay/msgs/.gitkeep`

- [ ] **Step 1: Create overlay directory structure**

```bash
mkdir -p webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap
mkdir -p webapp/scrimmage-overlay/src/plugins/interaction/TopicTap
mkdir -p webapp/scrimmage-overlay/missions
mkdir -p webapp/scrimmage-overlay/msgs
touch webapp/scrimmage-overlay/include/c2overlay/plugins/.gitkeep
touch webapp/scrimmage-overlay/src/plugins/.gitkeep
touch webapp/scrimmage-overlay/missions/.gitkeep
touch webapp/scrimmage-overlay/msgs/.gitkeep
```

- [ ] **Step 2: Create `webapp/scrimmage-overlay/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.10)
project(c2overlay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(scrimmage REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROJECT_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PROJECT_PLUGIN_LIBS_DIR ${CMAKE_CURRENT_BINARY_DIR}/plugin_libs)

include_directories(${PROJECT_INC_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)

add_subdirectory(msgs)
add_subdirectory(src/plugins/interaction/TopicTap)

include(GenerateSetEnv)
GenerateSetEnv(
  SETUP_LOCAL_CONFIG_DIR OFF
  SETENV_IN_FILE ${SCRIMMAGE_CMAKE_MODULES}/setenv.in
  MISSION_PATH ${PROJECT_SOURCE_DIR}/missions
  PLUGIN_PATH ${PROJECT_PLUGIN_LIBS_DIR}
              ${PROJECT_SOURCE_DIR}/include/c2overlay/plugins
)

install(DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}/
  DESTINATION lib/c2overlay/plugin_libs
  FILES_MATCHING PATTERN "*.so")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/missions/
  DESTINATION share/c2overlay/missions)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/c2overlay/plugins/
  DESTINATION etc/c2overlay/plugins
  FILES_MATCHING PATTERN "*.xml")
```

- [ ] **Step 3: Create `webapp/scrimmage-overlay/.gitignore`**

```
build/
install/
*.pb.cc
*.pb.h
*.grpc.pb.cc
*.grpc.pb.h
```

- [ ] **Step 4: Commit**

```bash
git add webapp/scrimmage-overlay/
git commit -m "feat(overlay): scaffold SCRIMMAGE external-project overlay for TopicTap"
```

---

## Task 2: TopicTap protobuf service

**Files:**
- Create: `webapp/scrimmage-overlay/msgs/CMakeLists.txt`
- Create: `webapp/scrimmage-overlay/msgs/TopicTap.proto`

- [ ] **Step 1: Create `webapp/scrimmage-overlay/msgs/TopicTap.proto`**

```proto
syntax = "proto3";

package c2overlay_msgs;

service TopicTapService {
  rpc ListTopics(ListTopicsRequest) returns (TopicList);
  rpc StreamTopic(StreamTopicRequest) returns (stream TopicMessage);
}

message ListTopicsRequest {}

message TopicSpec {
  string network = 1;
  string topic = 2;
  string type_name = 3;
}

message TopicList {
  repeated TopicSpec topics = 1;
}

message StreamTopicRequest {
  string network = 1;
  string topic = 2;
}

message TopicMessage {
  string network = 1;
  string topic = 2;
  string type_name = 3;
  double t_sim = 4;
  string payload_json = 5;
}
```

- [ ] **Step 2: Create `webapp/scrimmage-overlay/msgs/CMakeLists.txt`**

```cmake
set(PROTO_SRCS TopicTap.proto)
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(_protoc $<TARGET_FILE:protobuf::protoc>)
set(_grpc_cpp_plugin $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

set(GENERATED_SRCS "")
foreach(proto IN LISTS PROTO_SRCS)
  get_filename_component(name ${proto} NAME_WE)
  set(pb_cc ${GENERATED_DIR}/${name}.pb.cc)
  set(pb_h  ${GENERATED_DIR}/${name}.pb.h)
  set(grpc_cc ${GENERATED_DIR}/${name}.grpc.pb.cc)
  set(grpc_h  ${GENERATED_DIR}/${name}.grpc.pb.h)
  add_custom_command(
    OUTPUT ${pb_cc} ${pb_h} ${grpc_cc} ${grpc_h}
    COMMAND ${_protoc}
    ARGS --grpc_out=${GENERATED_DIR}
         --cpp_out=${GENERATED_DIR}
         -I${CMAKE_CURRENT_SOURCE_DIR}
         --plugin=protoc-gen-grpc=${_grpc_cpp_plugin}
         ${CMAKE_CURRENT_SOURCE_DIR}/${proto}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${proto})
  list(APPEND GENERATED_SRCS ${pb_cc} ${grpc_cc})
endforeach()

add_library(c2overlay-msgs STATIC ${GENERATED_SRCS})
target_include_directories(c2overlay-msgs PUBLIC ${GENERATED_DIR})
target_link_libraries(c2overlay-msgs PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
)
```

- [ ] **Step 3: Commit**

```bash
git add webapp/scrimmage-overlay/msgs/
git commit -m "feat(topictap): add gRPC service definition for TopicTap bridge"
```

---

## Task 3: TopicTap plugin headers

**Files:**
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTap.h`
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTapServiceImpl.h`

- [ ] **Step 1: Create `TopicTap.h`**

```cpp
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
```

- [ ] **Step 2: Create `TopicTapServiceImpl.h`**

```cpp
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

 private:
  TopicTap* parent_;  // not owned
};

}  // namespace interaction
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_INTERACTION_TOPICTAP_TOPICTAPSERVICEIMPL_H_
```

- [ ] **Step 3: Commit**

```bash
git add webapp/scrimmage-overlay/include/
git commit -m "feat(topictap): plugin and gRPC service headers"
```

---

## Task 4: TopicTap plugin implementation

**Files:**
- Create: `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTap.cpp`
- Create: `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTapServiceImpl.cpp`
- Create: `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/CMakeLists.txt`
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTap.xml`

- [ ] **Step 1: Create `TopicTap.cpp`**

```cpp
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
```

- [ ] **Step 2: Create `TopicTapServiceImpl.cpp`**

```cpp
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

}  // namespace interaction
}  // namespace c2overlay
```

- [ ] **Step 3: Create `TopicTap.xml`** (plugin config the operator references in mission XML)

```xml
<?xml version="1.0"?>
<params>
  <library>TopicTap_plugin</library>

  <ip>0.0.0.0</ip>
  <port>60001</port>
</params>
```

- [ ] **Step 4: Create `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/CMakeLists.txt`**

```cmake
set(LIBRARY_NAME TopicTap_plugin)

set(SRCS
  TopicTap.cpp
  TopicTapServiceImpl.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRCS})

target_link_libraries(${LIBRARY_NAME}
  c2overlay-msgs
  scrimmage-core
  scrimmage-msgs
  protobuf::libprotobuf
  gRPC::grpc++
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INC_DIR}>
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
)
```

- [ ] **Step 5: Commit**

```bash
git add webapp/scrimmage-overlay/src/ webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTap.xml
git commit -m "feat(topictap): plugin implementation, gRPC service, and CMake build"
```

---

## Task 5: Mission XML — predator_prey_boids with TopicTap

The base image already ships `predator_prey_boids.xml`, but we need to layer in the `TopicTap` plugin reference. Put the modified copy in the overlay's `missions/` dir; SCRIMMAGE_MISSION_PATH ordering means the overlay copy wins.

**Files:**
- Create: `webapp/scrimmage-overlay/missions/predator_prey_boids.xml`

- [ ] **Step 1: Copy + edit the mission file**

Copy `missions/predator_prey_boids.xml` to `webapp/scrimmage-overlay/missions/predator_prey_boids.xml` and add the `TopicTap` entity_interaction. Final file content:

```xml
<runscript name="Predator Prey Mission with Boids">

  <run dt="0.1" enable_gui="${enable_gui=true}" end="150" network_gui="false" start="0.0" start_paused="true" time_warp="10"
    motion_multiplier="12"/>

  <end_condition>time, all_dead</end_condition>

  <grid_spacing>100</grid_spacing>
  <grid_size>10000</grid_size>

  <terrain>mcmillan</terrain>
  <background_color>191 191 191</background_color>
  <gui_update_period>10</gui_update_period>

  <plot_tracks>false</plot_tracks>
  <output_type>summary</output_type>
  <show_plugins>false</show_plugins>

  <log_dir>~/.scrimmage/logs</log_dir>

  <latitude_origin>35.721025</latitude_origin>
  <longitude_origin>-120.767925</longitude_origin>
  <altitude_origin>300</altitude_origin>
  <show_origin>true</show_origin>
  <origin_length>5</origin_length>

  <network>GlobalNetwork</network>
  <network>LocalNetwork</network>

  <entity_interaction capture_range="5">SimpleCapture</entity_interaction>
  <entity_interaction startup_collisions_only="true">SimpleCollision</entity_interaction>
  <entity_interaction
    tap_network="GlobalNetwork"
    tap_topic="CaptureEntity"
    tap_type="scrimmage_msgs.CaptureEntity">TopicTap</entity_interaction>

  <metrics>SimpleCaptureMetrics</metrics>

  <entity>
    <team_id>1</team_id>
    <color>0 0 255</color>
    <count>100</count>
    <health>5</health>
    <variance_x>400000</variance_x>
    <variance_y>400000</variance_y>
    <variance_z>1000</variance_z>
    <x>0</x>
    <y>0</y>
    <z>200</z>
    <heading>0</heading>
    <autonomy>Boids</autonomy>
    <motion_model>JSBSimModel</motion_model>
    <controller>JSBSimModelControllerHeadingPID</controller>
    <script_name>rascal_piedmont.xml</script_name>
    <visual_model>zephyr-blue</visual_model>
  </entity>

  <entity_common name="predator">
    <health>5</health>
    <x>1000</x>
    <y>0</y>
    <z>200</z>
    <heading>180</heading>
    <motion_model pitch_rate_max="1.5" turn_rate_max="2" vel_max="35" use_pitch="true">Unicycle</motion_model>
    <controller>DirectController</controller>
    <visual_model>zephyr-red</visual_model>
  </entity_common>

  <entity entity_common="predator">
    <team_id>2</team_id>
    <count>1</count>
    <color>255 0 0</color>
    <autonomy allow_prey_switching="true" capture_range="5" max_speed="35">Predator</autonomy>
  </entity>

</runscript>
```

(The flat `tap_network`/`tap_topic`/`tap_type` attributes match the parser in `TopicTap::init`. To tap multiple topics, use comma-separated values: `tap_network="GlobalNetwork,LocalNetwork"`, `tap_topic="A,B"`, `tap_type="...,..."`.)

- [ ] **Step 2: Commit**

```bash
git add webapp/scrimmage-overlay/missions/predator_prey_boids.xml
git commit -m "feat(missions): overlay copy of predator_prey_boids with TopicTap interaction"
```

---

## Task 6: Build the overlay inside the scrimmage container

Modify the existing `scrimmage-runner` Dockerfile to copy the overlay source in, build it, install, and source the resulting setenv.

**Files:**
- Modify: `webapp/scrimmage-runner/Dockerfile`
- Modify: `webapp/scrimmage-runner/entrypoint.sh`

- [ ] **Step 1: Edit `webapp/scrimmage-runner/Dockerfile`**

Add a build stage *before* the final `ENTRYPOINT` line. The final file:

```dockerfile
FROM ghcr.io/gtri/scrimmage-24.04:latest

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv \
        cmake g++ make \
        protobuf-compiler protobuf-compiler-grpc \
        libprotobuf-dev libgrpc++-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/launcher-venv

ENV SCRIMMAGE_ROOT=/root/scrimmage/build/install
ENV PATH="${SCRIMMAGE_ROOT}/bin:/opt/launcher-venv/bin:${PATH}"
ENV LD_LIBRARY_PATH="${SCRIMMAGE_ROOT}/lib"
ENV SCRIMMAGE_PLUGIN_PATH="${SCRIMMAGE_ROOT}/lib/scrimmage/plugin_libs:${SCRIMMAGE_ROOT}/etc/scrimmage/plugins"
ENV SCRIMMAGE_MISSION_PATH="${SCRIMMAGE_ROOT}/share/scrimmage/missions"
ENV SCRIMMAGE_DATA_PATH="${SCRIMMAGE_ROOT}/share/scrimmage/data"
ENV SCRIMMAGE_CONFIG_PATH="${SCRIMMAGE_ROOT}/etc/scrimmage"

ENV C2OVERLAY_ROOT=/opt/c2overlay/install
ENV SCRIMMAGE_PLUGIN_PATH="${C2OVERLAY_ROOT}/lib/c2overlay/plugin_libs:${C2OVERLAY_ROOT}/etc/c2overlay/plugins:${SCRIMMAGE_PLUGIN_PATH}"
ENV LD_LIBRARY_PATH="${C2OVERLAY_ROOT}/lib/c2overlay/plugin_libs:${LD_LIBRARY_PATH}"

# Build the overlay. Note: the docker compose build context must be `webapp/`
# (set in Task 7) so this COPY resolves correctly.
COPY scrimmage-overlay /opt/c2overlay/src
RUN mkdir -p /opt/c2overlay/build && cd /opt/c2overlay/build \
    && cmake /opt/c2overlay/src \
         -DSETUP_LOCAL_CONFIG_DIR=OFF \
         -DCMAKE_INSTALL_PREFIX=${C2OVERLAY_ROOT} \
         -Dscrimmage_DIR=${SCRIMMAGE_ROOT}/share/cmake/scrimmage \
    && make -j$(nproc) install

# Mission dir the launcher reads from. Keep MISSIONS_DIR as the MVP set it up,
# but copy our overlay missions into it so predator_prey_boids.xml (with TopicTap)
# overrides the base-image copy.
ENV MISSIONS_DIR=/root/scrimmage/missions
RUN mkdir -p "$MISSIONS_DIR" \
    && cp -f ${C2OVERLAY_ROOT}/share/c2overlay/missions/*.xml "$MISSIONS_DIR/"

COPY launcher/requirements.txt /opt/launcher/requirements.txt
RUN pip install --no-cache-dir -r /opt/launcher/requirements.txt

COPY launcher/ /opt/launcher/
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /opt

EXPOSE 50051 5050 60001

ENTRYPOINT ["/opt/entrypoint.sh"]
```

**IMPORTANT:** the `COPY ../scrimmage-overlay /opt/c2overlay/src` line requires the docker compose `build.context` to point at `webapp/` (not `webapp/scrimmage-runner/`). Task 8 updates that.

- [ ] **Step 2: Verify entrypoint.sh works as-is**

Read `webapp/scrimmage-runner/entrypoint.sh`. If it just `exec`s the launcher, no changes needed — the env vars are baked in via Dockerfile. If it `source`s setenv files, add lines to also source the overlay. (For the MVP-shipped entrypoint, no edit is expected; verify and skip the edit if so.)

- [ ] **Step 3: Commit**

```bash
git add webapp/scrimmage-runner/Dockerfile
git commit -m "feat(scrimmage-runner): build c2overlay with TopicTap plugin into the image"
```

---

## Task 7: docker-compose updates

**Files:**
- Modify: `webapp/docker-compose.yml`

- [ ] **Step 1: Edit `webapp/docker-compose.yml`**

Two changes to the `scrimmage` service: build context moves up one level so the Dockerfile can `COPY` the overlay; add port `60001` to `ports`.

```yaml
  scrimmage:
    build:
      context: .
      dockerfile: scrimmage-runner/Dockerfile
    container_name: c2-scrimmage
    ports:
      - "50051:50051"   # gRPC frame stream
      - "5050:5050"     # launcher HTTP
      - "60001:60001"   # gRPC TopicTap (debug; not strictly needed since api hits it via internal network)
    networks: [c2net]
```

Then add the env var to the `api` service:

```yaml
  api:
    # ...existing fields...
    environment:
      - ASPNETCORE_URLS=http://+:8080
      - SCRIMMAGE_GRPC_ADDR=http://scrimmage:50051
      - SCRIMMAGE_LAUNCHER_URL=http://scrimmage:5050
      - TOPICTAP_GRPC_ADDR=http://scrimmage:60001
      - ConnectionStrings__Default=Host=postgres;Database=c2;Username=c2;Password=c2pass
```

- [ ] **Step 2: Commit**

```bash
git add webapp/docker-compose.yml
git commit -m "feat(compose): expose TopicTap port and pass address to api"
```

---

## Task 8: Mirror TopicTap.proto into the API

**Files:**
- Create: `webapp/api/Protos/scrimmage/TopicTap.proto`
- Modify: `webapp/api/Api.csproj`

- [ ] **Step 1: Create `webapp/api/Protos/scrimmage/TopicTap.proto`**

Identical content to the overlay's TopicTap.proto, with an explicit `csharp_namespace` option so the C# import in Task 11 is unambiguous:

```proto
syntax = "proto3";

package c2overlay_msgs;

option csharp_namespace = "ScrimmageC2.Generated.TopicTap";

service TopicTapService {
  rpc ListTopics(ListTopicsRequest) returns (TopicList);
  rpc StreamTopic(StreamTopicRequest) returns (stream TopicMessage);
}

message ListTopicsRequest {}

message TopicSpec {
  string network = 1;
  string topic = 2;
  string type_name = 3;
}

message TopicList {
  repeated TopicSpec topics = 1;
}

message StreamTopicRequest {
  string network = 1;
  string topic = 2;
}

message TopicMessage {
  string network = 1;
  string topic = 2;
  string type_name = 3;
  double t_sim = 4;
  string payload_json = 5;
}
```

- [ ] **Step 2: Edit `webapp/api/Api.csproj`**

Add a new `<Protobuf>` item alongside the existing ones (you should see other `<Protobuf Include="Protos/scrimmage/*.proto" .../>` entries already; mirror the pattern):

```xml
<ItemGroup>
  <Protobuf Include="Protos/scrimmage/TopicTap.proto" GrpcServices="Client" />
</ItemGroup>
```

If the existing protos already use a glob, this entry is redundant and the new file is picked up automatically — verify by checking the existing item group.

- [ ] **Step 3: Verify the API project still builds**

```bash
cd webapp/api && dotnet build
```

Expected: `Build succeeded`. If `dotnet` isn't on the host PATH, run inside the api container instead: `docker compose -f webapp/docker-compose.yml build api`.

- [ ] **Step 4: Commit**

```bash
git add webapp/api/Protos/scrimmage/TopicTap.proto webapp/api/Api.csproj
git commit -m "feat(api): mirror TopicTap.proto for client-side gRPC codegen"
```

---

## Task 9: API DTOs

**Files:**
- Modify: `webapp/api/Dtos.cs`

- [ ] **Step 1: Add new DTOs at the bottom of `webapp/api/Dtos.cs`**

```csharp
public sealed record TopicSpecDto(string Network, string Topic, string TypeName);

public sealed record TopicMessageDto(
    string Network,
    string Topic,
    string TypeName,
    double TSim,
    string PayloadJson);
```

- [ ] **Step 2: Commit**

```bash
git add webapp/api/Dtos.cs
git commit -m "feat(api): add TopicSpecDto and TopicMessageDto"
```

---

## Task 10: TopicHub (SignalR)

**Files:**
- Create: `webapp/api/TopicHub.cs`

- [ ] **Step 1: Create `webapp/api/TopicHub.cs`**

```csharp
using Microsoft.AspNetCore.SignalR;

namespace ScrimmageC2.Api;

public sealed class TopicHub : Hub
{
    private readonly TopicState _state;

    public TopicHub(TopicState state)
    {
        _state = state;
    }

    public override async Task OnConnectedAsync()
    {
        // Replay current topic list to the new client so the dropdown is populated
        // without a separate REST round-trip.
        await Clients.Caller.SendAsync("OnTopicList", _state.Topics);
        await base.OnConnectedAsync();
    }
}

// Singleton holding the latest list of configured topics seen from the sim.
public sealed class TopicState
{
    private readonly object _lock = new();
    private IReadOnlyList<TopicSpecDto> _topics = Array.Empty<TopicSpecDto>();

    public IReadOnlyList<TopicSpecDto> Topics
    {
        get { lock (_lock) return _topics; }
    }

    public void SetTopics(IReadOnlyList<TopicSpecDto> topics)
    {
        lock (_lock) _topics = topics;
    }
}
```

(Adjust the `namespace` to match the namespace in your existing API files — likely `Api` or whatever `Program.cs` uses. Use the same throughout.)

- [ ] **Step 2: Commit**

```bash
git add webapp/api/TopicHub.cs
git commit -m "feat(api): TopicHub SignalR hub + TopicState singleton"
```

---

## Task 11: TopicTapClient (BackgroundService)

**Files:**
- Create: `webapp/api/TopicTapClient.cs`

- [ ] **Step 1: Create `webapp/api/TopicTapClient.cs`**

```csharp
using ScrimmageC2.Generated.TopicTap;  // matches csharp_namespace option in Protos/scrimmage/TopicTap.proto
using Grpc.Core;
using Grpc.Net.Client;
using Microsoft.AspNetCore.SignalR;

namespace ScrimmageC2.Api;

public sealed class TopicTapClient : BackgroundService
{
    private readonly IHubContext<TopicHub> _hub;
    private readonly TopicState _state;
    private readonly ILogger<TopicTapClient> _log;
    private readonly string _addr;

    public TopicTapClient(
        IHubContext<TopicHub> hub,
        TopicState state,
        IConfiguration config,
        ILogger<TopicTapClient> log)
    {
        _hub = hub;
        _state = state;
        _log = log;
        _addr = config["TOPICTAP_GRPC_ADDR"] ?? "http://scrimmage:60001";
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await RunOnce(stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception ex)
            {
                _log.LogWarning(ex, "TopicTap connection failed; retrying in 500ms");
                _state.SetTopics(Array.Empty<TopicSpecDto>());
                await SafeBroadcastTopicList();
            }
            try { await Task.Delay(500, stoppingToken); }
            catch (OperationCanceledException) { return; }
        }
    }

    private async Task RunOnce(CancellationToken ct)
    {
        using var channel = GrpcChannel.ForAddress(_addr);
        var client = new TopicTapService.TopicTapServiceClient(channel);

        var topicList = await client.ListTopicsAsync(new ListTopicsRequest(), cancellationToken: ct);
        var dtos = topicList.Topics
            .Select(t => new TopicSpecDto(t.Network, t.Topic, t.TypeName))
            .ToList();
        _state.SetTopics(dtos);
        await _hub.Clients.All.SendAsync("OnTopicList", dtos, ct);

        if (dtos.Count == 0)
        {
            _log.LogInformation("TopicTap connected but no topics configured");
            // Idle wait — sim is up but mission has no TopicTap taps.
            await Task.Delay(Timeout.Infinite, ct);
            return;
        }

        var streamTasks = dtos
            .Select(t => StreamOne(client, t, ct))
            .ToArray();
        await Task.WhenAll(streamTasks);
    }

    private async Task StreamOne(
        TopicTapService.TopicTapServiceClient client,
        TopicSpecDto spec,
        CancellationToken ct)
    {
        var req = new StreamTopicRequest { Network = spec.Network, Topic = spec.Topic };
        using var call = client.StreamTopic(req, cancellationToken: ct);
        await foreach (var msg in call.ResponseStream.ReadAllAsync(ct))
        {
            var dto = new TopicMessageDto(
                msg.Network,
                msg.Topic,
                msg.TypeName,
                msg.TSim,
                msg.PayloadJson);
            await _hub.Clients.All.SendAsync("OnTopicMessage", dto, ct);
        }
    }

    private async Task SafeBroadcastTopicList()
    {
        try { await _hub.Clients.All.SendAsync("OnTopicList", _state.Topics); }
        catch { /* hub may not be up yet; benign */ }
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add webapp/api/TopicTapClient.cs
git commit -m "feat(api): TopicTapClient background service that bridges sim to SignalR"
```

---

## Task 12: REST endpoint + Program.cs wiring

**Files:**
- Modify: `webapp/api/Endpoints.cs`
- Modify: `webapp/api/Program.cs`

- [ ] **Step 1: Add `GET /api/topics` to `webapp/api/Endpoints.cs`**

Find the existing endpoint registration (where `MapGet("/api/missions", ...)` lives) and add:

```csharp
app.MapGet("/api/topics", (TopicState state) => Results.Ok(state.Topics));
```

(You need a `using ScrimmageC2.Api;` at the top of `Endpoints.cs` if it's not already there, matching the namespace from Task 10.)

- [ ] **Step 2: Wire services + hub in `webapp/api/Program.cs`**

Add to the DI container registration block (where `builder.Services.AddSignalR()` is called):

```csharp
builder.Services.AddSingleton<TopicState>();
builder.Services.AddHostedService<TopicTapClient>();
```

Add to the endpoint mapping block (where `app.MapHub<FrameHub>("/hubs/frames")` is called):

```csharp
app.MapHub<TopicHub>("/hubs/topics");
```

- [ ] **Step 3: Build to verify**

```bash
cd webapp/api && dotnet build
```

Expected: `Build succeeded` with zero errors. Warnings about nullable reference types are acceptable.

- [ ] **Step 4: Commit**

```bash
git add webapp/api/Endpoints.cs webapp/api/Program.cs
git commit -m "feat(api): wire TopicTapClient, TopicHub, and GET /api/topics"
```

---

## Task 13: Web — types and SignalR hook

**Files:**
- Modify: `webapp/web/src/types.ts`
- Create: `webapp/web/src/hooks/useTopicStream.ts`

- [ ] **Step 1: Add types to `webapp/web/src/types.ts`**

```typescript
export interface TopicSpec {
  network: string;
  topic: string;
  typeName: string;
}

export interface TopicMessage {
  network: string;
  topic: string;
  typeName: string;
  tSim: number;
  payloadJson: string;
}
```

- [ ] **Step 2: Create `webapp/web/src/hooks/useTopicStream.ts`**

```typescript
import { useEffect, useState } from "react";
import * as signalR from "@microsoft/signalr";
import type { TopicSpec, TopicMessage } from "../types";

const MAX_MESSAGES_PER_TOPIC = 100;

interface State {
  topics: TopicSpec[];
  messagesByKey: Map<string, TopicMessage[]>;
}

const key = (network: string, topic: string) => `${network}:${topic}`;

let connection: signalR.HubConnection | null = null;
let stateRef: State = { topics: [], messagesByKey: new Map() };
const subscribers = new Set<(s: State) => void>();

function notify() {
  for (const sub of subscribers) sub(stateRef);
}

function ensureConnection() {
  if (connection) return connection;
  connection = new signalR.HubConnectionBuilder()
    .withUrl("/hubs/topics")
    .withAutomaticReconnect()
    .build();

  connection.on("OnTopicList", (topics: TopicSpec[]) => {
    stateRef = {
      topics,
      // New session boundary: clear feeds.
      messagesByKey: new Map(),
    };
    notify();
  });

  connection.on("OnTopicMessage", (msg: TopicMessage) => {
    const k = key(msg.network, msg.topic);
    const existing = stateRef.messagesByKey.get(k) ?? [];
    const next = [msg, ...existing].slice(0, MAX_MESSAGES_PER_TOPIC);
    const newMap = new Map(stateRef.messagesByKey);
    newMap.set(k, next);
    stateRef = { ...stateRef, messagesByKey: newMap };
    notify();
  });

  connection.start().catch(err => console.error("TopicHub start failed", err));
  return connection;
}

export function useTopicStream() {
  const [snapshot, setSnapshot] = useState<State>(stateRef);

  useEffect(() => {
    ensureConnection();
    subscribers.add(setSnapshot);
    return () => {
      subscribers.delete(setSnapshot);
    };
  }, []);

  return {
    topics: snapshot.topics,
    messagesFor: (network: string, topic: string): TopicMessage[] =>
      snapshot.messagesByKey.get(key(network, topic)) ?? [],
  };
}
```

- [ ] **Step 3: Commit**

```bash
git add webapp/web/src/types.ts webapp/web/src/hooks/useTopicStream.ts
git commit -m "feat(web): TopicSpec/TopicMessage types and useTopicStream hook"
```

---

## Task 14: Web — TopicFeedPanel component

**Files:**
- Create: `webapp/web/src/components/panels/TopicFeedPanel.tsx`

- [ ] **Step 1: Create `webapp/web/src/components/panels/TopicFeedPanel.tsx`**

```tsx
import { useEffect, useMemo, useState } from "react";
import { useTopicStream } from "../../hooks/useTopicStream";
import type { TopicMessage } from "../../types";

type Formatter = (msg: TopicMessage) => string;

const FORMATTERS: Record<string, Formatter> = {
  "scrimmage_msgs.CaptureEntity": (msg) => {
    try {
      const p = JSON.parse(msg.payloadJson);
      return `t=${msg.tSim.toFixed(2)}s — entity ${p.sourceId} captured entity ${p.targetId}`;
    } catch {
      return `t=${msg.tSim.toFixed(2)}s — ${msg.payloadJson}`;
    }
  },
};

function formatMessage(msg: TopicMessage): string {
  const fmt = FORMATTERS[msg.typeName];
  if (fmt) return fmt(msg);
  try {
    const p = JSON.parse(msg.payloadJson);
    return `t=${msg.tSim.toFixed(2)}s — ${JSON.stringify(p)}`;
  } catch {
    return `t=${msg.tSim.toFixed(2)}s — ${msg.payloadJson}`;
  }
}

export function TopicFeedPanel() {
  const { topics, messagesFor } = useTopicStream();
  const [selected, setSelected] = useState<string>("");

  // Default-select the first topic when the list arrives or changes.
  useEffect(() => {
    if (topics.length === 0) {
      setSelected("");
    } else if (!topics.some(t => `${t.network}:${t.topic}` === selected)) {
      setSelected(`${topics[0].network}:${topics[0].topic}`);
    }
  }, [topics, selected]);

  const messages = useMemo(() => {
    if (!selected) return [];
    const [network, topic] = selected.split(":", 2);
    return messagesFor(network, topic);
  }, [selected, messagesFor]);

  if (topics.length === 0) {
    return (
      <section className="panel topic-feed">
        <header className="panel__header">Topics</header>
        <div className="panel__body panel__body--empty">
          No topics configured for this mission.
        </div>
      </section>
    );
  }

  return (
    <section className="panel topic-feed">
      <header className="panel__header">Topics</header>
      <div className="panel__controls">
        <select value={selected} onChange={e => setSelected(e.target.value)}>
          {topics.map(t => {
            const k = `${t.network}:${t.topic}`;
            return <option key={k} value={k}>{t.network} / {t.topic}</option>;
          })}
        </select>
      </div>
      <ul className="panel__feed">
        {messages.length === 0 ? (
          <li className="panel__feed-empty">Waiting for messages on {selected}…</li>
        ) : (
          messages.map((msg, i) => (
            <li key={`${msg.tSim}-${i}`}>{formatMessage(msg)}</li>
          ))
        )}
      </ul>
    </section>
  );
}
```

(The `panel__*` class names assume the existing AppShell uses a similar BEM-ish convention. If not, adjust to whatever class structure the existing stub panels use — the visual fidelity is "fits the existing right-rail look," not pixel-perfect.)

- [ ] **Step 2: Commit**

```bash
git add webapp/web/src/components/panels/TopicFeedPanel.tsx
git commit -m "feat(web): TopicFeedPanel with topic dropdown and scrolling event feed"
```

---

## Task 15: Web — slot TopicFeedPanel into AppShell

**Files:**
- Modify: `webapp/web/src/AppShell.tsx`

- [ ] **Step 1: Replace the stub topics panel in `webapp/web/src/AppShell.tsx`**

Find the placeholder topics panel in the right rail (per the MVP spec it's a stub). Import:

```tsx
import { TopicFeedPanel } from "./components/panels/TopicFeedPanel";
```

Replace the stub element (likely `<section className="panel">Topics (stub)</section>` or similar) with:

```tsx
<TopicFeedPanel />
```

- [ ] **Step 2: Commit**

```bash
git add webapp/web/src/AppShell.tsx
git commit -m "feat(web): wire TopicFeedPanel into the operator console right rail"
```

---

## Task 16: C++ unit test for the JSON encoder

The only unit test the spec asks for: round-trip a `CaptureEntity` proto through `MessageToJsonString` and confirm the output matches what the operator will see. Catches a silent encoder-misconfig regression.

**Files:**
- Create: `webapp/scrimmage-overlay/test/CMakeLists.txt`
- Create: `webapp/scrimmage-overlay/test/test_capture_entity_json.cpp`
- Modify: `webapp/scrimmage-overlay/CMakeLists.txt`

- [ ] **Step 1: Create `webapp/scrimmage-overlay/test/test_capture_entity_json.cpp`**

```cpp
#include <cassert>
#include <iostream>
#include <string>

#include <google/protobuf/util/json_util.h>
#include "scrimmage/msgs/Capture.pb.h"

int main() {
  scrimmage_msgs::CaptureEntity msg;
  msg.set_source_id(5);
  msg.set_target_id(47);

  std::string out;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = false;
  auto status = google::protobuf::util::MessageToJsonString(msg, &out, opts);
  if (!status.ok()) {
    std::cerr << "MessageToJsonString failed: " << status.message() << "\n";
    return 1;
  }

  // Default JSON casing converts source_id -> sourceId, target_id -> targetId.
  const std::string expected = "{\"sourceId\":5,\"targetId\":47}";
  if (out != expected) {
    std::cerr << "Mismatch.\n  expected: " << expected << "\n  got:      " << out << "\n";
    return 1;
  }
  std::cout << "OK: " << out << "\n";
  return 0;
}
```

- [ ] **Step 2: Create `webapp/scrimmage-overlay/test/CMakeLists.txt`**

```cmake
add_executable(test_capture_entity_json test_capture_entity_json.cpp)
target_link_libraries(test_capture_entity_json
  scrimmage-msgs
  protobuf::libprotobuf
)
add_test(NAME test_capture_entity_json COMMAND test_capture_entity_json)
```

- [ ] **Step 3: Edit `webapp/scrimmage-overlay/CMakeLists.txt`**

Add at the bottom (before the install rules):

```cmake
enable_testing()
add_subdirectory(test)
```

- [ ] **Step 4: Verify the test passes inside the container**

```bash
docker compose -f webapp/docker-compose.yml build scrimmage
docker compose -f webapp/docker-compose.yml run --rm scrimmage \
  bash -c 'cd /opt/c2overlay/build && ctest --output-on-failure'
```

Expected: `100% tests passed, 0 tests failed out of 1`.

- [ ] **Step 5: Commit**

```bash
git add webapp/scrimmage-overlay/test/ webapp/scrimmage-overlay/CMakeLists.txt
git commit -m "test(topictap): unit test for CaptureEntity JSON serialization"
```

---

## Task 17: Manual smoke test (acceptance)

This is the *only* test that absolutely must pass before declaring v1 done. Per the spec.

- [ ] **Step 1: Clean rebuild**

```bash
cd webapp
docker compose down -v
docker compose up --build
```

Expected: all four containers start. `c2-scrimmage` log shows `[TopicTap] gRPC listening on 0.0.0.0:60001` after the launcher boots.

- [ ] **Step 2: Open browser**

Navigate to `http://localhost:5173`.

- [ ] **Step 3: Verify mission dropdown**

The MissionPicker dropdown should include `predator_prey_boids.xml` (it's in the overlay's missions dir, which is on `SCRIMMAGE_MISSION_PATH`).

- [ ] **Step 4: Start the mission**

Select `predator_prey_boids.xml` → Start. Expect within ~3s: 100 blue boids + 1 red predator visible in Cesium over the Camp Roberts terrain.

- [ ] **Step 5: Verify the topic dropdown populates**

The right-rail Topics panel dropdown should show one entry: `GlobalNetwork / CaptureEntity`. (If it shows "No topics configured for this mission", check `docker logs c2-scrimmage` for `[TopicTap] tapped GlobalNetwork:CaptureEntity (scrimmage_msgs.CaptureEntity)`.)

- [ ] **Step 6: Verify capture events appear**

Within ~10–30s (depends on predator catching prey), entries should appear in the feed:

```
t=12.34s — entity 101 captured entity 47
t=14.71s — entity 101 captured entity 33
...
```

Each event should line up with a blue boid disappearing from the Cesium view.

- [ ] **Step 7: Verify mission switch clears the feed**

Click Stop → feed stays (frozen). Switch to `capture-the-flag.xml` → Start. Expect: the feed clears and the panel shows "No topics configured for this mission" (CTF doesn't load TopicTap).

- [ ] **Step 8: Verify reconnect**

Switch back to `predator_prey_boids.xml` → Start. Refresh the browser mid-mission. Expect: dropdown re-populates within ~1s and new captures resume appending.

- [ ] **Step 9: Tag the release**

```bash
git tag topictap-v1
```

(No commit needed — all work was committed in earlier tasks.)

---

## Self-review checklist

After all tasks complete, verify:

- [ ] `git log --oneline | head -20` shows ~16 commits, all with `feat(...)` or `test(...)` prefixes.
- [ ] Spec section 4.1 (`TopicTap` plugin) — covered by tasks 3, 4.
- [ ] Spec section 4.2 (`TopicTapClient`) — task 11.
- [ ] Spec section 4.3 (`TopicHub`) — task 10.
- [ ] Spec section 4.4 (REST endpoint) — task 12.
- [ ] Spec section 4.5 (`TopicFeedPanel`) — tasks 14, 15.
- [ ] Spec section 4.6 (`useTopicStream`) — task 13.
- [ ] Spec section 4.7 (mission XML edit) — task 5.
- [ ] Spec section 5 (wire formats) — tasks 2, 8 (proto on both sides), 9 (DTOs).
- [ ] Spec section 7 (error handling) — implemented in task 11 (retry loop), task 4 (ring buffer with drop-oldest).
- [ ] Spec section 8.2 (one unit test) — task 16.
- [ ] Spec section 8.1 (manual smoke) — task 17.

If any spec section is uncovered, add a task before declaring complete.
