# Target Assignment Command (v1) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the operator's first command — **target assignment** — so they can pick a prey boid in the EntityInspectorCard and have the predator hunt that specific entity. Light up the inject side of TopicTap (the v2 deferred from pub/sub bridge work) along the way; the typed `target-assignment` endpoint is sugar over a generic `PublishToTopic` path that future commands reuse for free.

**Architecture:** Extend `TopicTap` with a `PublishToTopic(network, topic, payload_json)` unary gRPC backed by a static `publish_registry`. RPC enqueues from the gRPC thread; `step_entity_interaction` drains and publishes from the sim thread (mirrors upstream `GRPCCommandString` for thread-safety). New overlay autonomy plugin `RemotePredator` (copy-and-modify of upstream `Predator`) subscribes to `Commands/TargetAssignment` and hard-locks `follow_id_` when an assignment matches its entity id. New API endpoint pair (generic `POST /api/topics/{net}/{topic}/publish` + typed sugar `POST /api/commands/target-assignment`). New "Set / Clear predator target" button on `EntityInspectorCard`.

**Tech Stack:** C++ + CMake + protobuf + gRPC (overlay plugin and autonomy); .NET 10 + `Grpc.Net.Client` (API endpoints + gRPC client); React + TypeScript (UI).

**Working directory:** All `git` commands run from `C:\Git\kinetas\scrimmage\` (the SCRIMMAGE fork). Use `git -C C:/Git/kinetas/scrimmage <cmd>` — the parent `C:/Git/kinetas/` is not a git repo. Do NOT use `cd`. Branch: `pubsub-plugin` (continues the TopicTap work).

**Spec:** `webapp/docs/specs/2026-05-02-target-assignment-design.md`

---

## Pre-flight

Before starting, verify:

- [ ] TopicTap v1 plan (`webapp/docs/plans/2026-05-02-pubsub-bridge-topictap-v1.md`) is implemented and `docker compose up --build` from `webapp/` succeeds with `predator_prey_boids.xml` showing live capture events in the right-rail topic feed.
- [ ] `webapp/web/src/components/EntityInspectorCard.tsx` exists (the user's parallel drone-inspector work).
- [ ] `git -C C:/Git/kinetas/scrimmage status` shows a clean tree on branch `pubsub-plugin`.
- [ ] Upstream `src/plugins/autonomy/Predator/Predator.cpp` exists at the expected path (used as the source for the RemotePredator copy in Task 4).

---

## Task 1: New `Commands.proto` (overlay + API mirror)

**Files:**
- Create: `webapp/scrimmage-overlay/msgs/Commands.proto`
- Modify: `webapp/scrimmage-overlay/msgs/CMakeLists.txt`
- Create: `webapp/api/Protos/scrimmage/Commands.proto` (mirror with `csharp_namespace`)
- Modify: `webapp/api/Api.csproj`

- [ ] **Step 1: Create `webapp/scrimmage-overlay/msgs/Commands.proto`** with EXACT content:

```proto
syntax = "proto3";

package c2overlay_msgs;

message TargetAssignment {
  int32 predator_id = 1;       // entity id of the predator to command; > 0
  int32 target_id = 2;          // entity id of the target; 0 = clear assignment
}
```

- [ ] **Step 2: Edit `webapp/scrimmage-overlay/msgs/CMakeLists.txt`** to compile the new proto.

Find the `set(PROTO_SRCS TopicTap.proto)` line (line 1) and change to:

```cmake
set(PROTO_SRCS TopicTap.proto Commands.proto)
```

Everything else in that file is fine — the `foreach` loop already handles multiple protos.

- [ ] **Step 3: Create `webapp/api/Protos/scrimmage/Commands.proto`** (mirror with explicit C# namespace) with EXACT content:

```proto
syntax = "proto3";

package c2overlay_msgs;

option csharp_namespace = "ScrimmageC2.Generated.Commands";

message TargetAssignment {
  int32 predator_id = 1;       // > 0
  int32 target_id = 2;          // 0 = clear, > 0 = assign
}
```

- [ ] **Step 4: Edit `webapp/api/Api.csproj`** to register the new proto.

Find the `<ItemGroup>` containing the existing `<Protobuf>` entries. Add:

```xml
<Protobuf Include="Protos\scrimmage\Commands.proto" GrpcServices="None" ProtoRoot="Protos" />
```

(`GrpcServices="None"` because Commands.proto only defines messages, no service.)

- [ ] **Step 5: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/msgs/Commands.proto webapp/scrimmage-overlay/msgs/CMakeLists.txt webapp/api/Protos/scrimmage/Commands.proto webapp/api/Api.csproj
git -C C:/Git/kinetas/scrimmage commit -m "feat(commands): add TargetAssignment protobuf message (overlay + API mirror)"
```

---

## Task 2: Extend `TopicTap.proto` with `PublishToTopic` RPC

**Files:**
- Modify: `webapp/scrimmage-overlay/msgs/TopicTap.proto`
- Modify: `webapp/api/Protos/scrimmage/TopicTap.proto`

- [ ] **Step 1: Edit `webapp/scrimmage-overlay/msgs/TopicTap.proto`**

Find the `service TopicTapService { ... }` block. Add a third RPC:

```proto
service TopicTapService {
  rpc ListTopics(ListTopicsRequest) returns (TopicList);
  rpc StreamTopic(StreamTopicRequest) returns (stream TopicMessage);
  rpc PublishToTopic(PublishToTopicRequest) returns (PublishToTopicResponse);
}
```

Append two new message definitions at the bottom of the file:

```proto
message PublishToTopicRequest {
  string network = 1;
  string topic = 2;
  string payload_json = 3;
}

message PublishToTopicResponse {
  bool ok = 1;
  string error = 2;
}
```

- [ ] **Step 2: Apply the same edits to `webapp/api/Protos/scrimmage/TopicTap.proto`**

Identical RPC line and message definitions. Don't touch the existing `option csharp_namespace = "ScrimmageC2.Generated.TopicTap";` declaration.

- [ ] **Step 3: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/msgs/TopicTap.proto webapp/api/Protos/scrimmage/TopicTap.proto
git -C C:/Git/kinetas/scrimmage commit -m "feat(topictap): add PublishToTopic unary RPC (sim + API mirror)"
```

---

## Task 3: Extend TopicTap plugin with publish path (the C++ load-bearing piece)

**Files:**
- Modify: `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTap.h`
- Modify: `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTapServiceImpl.h`
- Modify: `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTap.cpp`
- Modify: `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTapServiceImpl.cpp`

- [ ] **Step 1: Edit `TopicTap.h` — add publish-path declarations**

After the `wait_for_message` method declaration (around line 47, just before the `void stop();` line), add:

```cpp
  // Publish-side interface (called from gRPC service thread).
  // Validates + parses payload, enqueues for sim-thread publish.
  // Returns (ok, error) — empty error on success.
  std::pair<bool, std::string> enqueue_publish(
      const std::string& network,
      const std::string& topic,
      const std::string& payload_json);
```

In the `private:` section, after the `void run_server();` line, add:

```cpp
  // Sim-thread side of publish path (called from step_entity_interaction).
  void drain_publish_queue();

  // Publish handler: parses JSON, builds typed message, returns serialized
  // form ready for the sim thread to publish. Returns nullptr on parse error
  // (with error written to *err_out).
  using PublishHandlerFn = std::function<
      bool(const std::string& payload_json,
           std::shared_ptr<scrimmage::MessageBase>* msg_out,
           std::string* err_out)>;
  static const std::map<std::string, PublishHandlerFn>& publish_registry();
```

In the data members section (after `std::map<std::string, std::shared_ptr<Queue>> queues_;` and its mutex), add:

```cpp
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
```

Top of file, ensure these includes are present (some already are):

```cpp
#include <utility>   // for std::pair
```

(Already in the header from `<utility>` not used; add it explicitly to be safe. Other needed headers — `<deque>`, `<functional>`, `<map>`, `<memory>`, `<mutex>`, `<string>` — are already there from Task 3 of the TopicTap plan.)

- [ ] **Step 2: Edit `TopicTapServiceImpl.h` — add PublishToTopic method**

After the `StreamTopic` declaration, add:

```cpp
  grpc::Status PublishToTopic(
      grpc::ServerContext* ctx,
      const c2overlay_msgs::PublishToTopicRequest* req,
      c2overlay_msgs::PublishToTopicResponse* resp) override;
```

- [ ] **Step 3: Edit `TopicTap.cpp` — add publish_registry, enqueue_publish, drain_publish_queue, modify step_entity_interaction**

At the top of the file, after the existing includes, add:

```cpp
#include "Commands.pb.h"
```

Below the existing `proto_to_json` template (around line 30), add a helper for the inverse direction:

```cpp
// Helper used by publish handlers: parse JSON into a typed protobuf.
template <class T>
static bool json_to_proto(const std::string& json, T* msg, std::string* err) {
  google::protobuf::util::JsonParseOptions opts;
  opts.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(json, msg, opts);
  if (!status.ok()) {
    *err = std::string("json parse: ") + std::string(status.message());
    return false;
  }
  return true;
}
```

After the existing `type_registry()` static method, add the publish_registry:

```cpp
const std::map<std::string, TopicTap::PublishHandlerFn>& TopicTap::publish_registry() {
  static const std::map<std::string, PublishHandlerFn> reg = {
      {"GlobalNetwork:Commands/TargetAssignment",
       [](const std::string& json,
          std::shared_ptr<scrimmage::MessageBase>* msg_out,
          std::string* err) -> bool {
         auto m = std::make_shared<scrimmage::Message<c2overlay_msgs::TargetAssignment>>();
         if (!json_to_proto(json, &m->data, err)) return false;
         *msg_out = m;
         return true;
       }},
      // Add new publishable topics here. Each addition requires a rebuild.
  };
  return reg;
}
```

After the existing `wait_for_message` method, add the publish-path methods:

```cpp
std::pair<bool, std::string> TopicTap::enqueue_publish(
    const std::string& network,
    const std::string& topic,
    const std::string& payload_json) {
  const auto& reg = publish_registry();
  std::string key = network + ":" + topic;
  auto it = reg.find(key);
  if (it == reg.end()) {
    return {false, "unknown publish topic '" + key + "'"};
  }
  std::shared_ptr<scrimmage::MessageBase> msg;
  std::string err;
  if (!it->second(payload_json, &msg, &err)) {
    return {false, err};
  }
  {
    std::lock_guard<std::mutex> g(pending_publishes_mutex_);
    pending_publishes_.push_back({network, topic, msg});
  }
  return {true, ""};
}

void TopicTap::drain_publish_queue() {
  std::deque<PendingPublish> local;
  {
    std::lock_guard<std::mutex> g(pending_publishes_mutex_);
    local.swap(pending_publishes_);
  }
  for (auto& pp : local) {
    std::string key = pp.network + ":" + pp.topic;
    auto it = pub_cache_.find(key);
    if (it == pub_cache_.end()) {
      pub_cache_[key] = advertise(pp.network, pp.topic);
    }
    pub_cache_[key]->publish(pp.msg);
  }
}
```

Modify `step_entity_interaction` to call the drain. Find the existing method (currently a no-op returning `true`) and replace its body:

```cpp
bool TopicTap::step_entity_interaction(
    std::list<scrimmage::EntityPtr>& /*ents*/,
    double /*t*/,
    double /*dt*/) {
  drain_publish_queue();
  return true;
}
```

- [ ] **Step 4: Edit `TopicTapServiceImpl.cpp` — implement PublishToTopic**

After the existing `StreamTopic` implementation, add:

```cpp
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
```

(C++17 structured binding is fine — TopicTap.cpp already uses C++17 features like `std::make_shared`.)

- [ ] **Step 5: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/ webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/
git -C C:/Git/kinetas/scrimmage commit -m "feat(topictap): publish path — PublishToTopic RPC + sim-thread drain via step_entity_interaction"
```

---

## Task 4: Create `RemotePredator` overlay autonomy plugin

**Files:**
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/autonomy/RemotePredator/RemotePredator.h`
- Create: `webapp/scrimmage-overlay/include/c2overlay/plugins/autonomy/RemotePredator/RemotePredator.xml`
- Create: `webapp/scrimmage-overlay/src/plugins/autonomy/RemotePredator/RemotePredator.cpp`
- Create: `webapp/scrimmage-overlay/src/plugins/autonomy/RemotePredator/CMakeLists.txt`

- [ ] **Step 1: Create `RemotePredator.h`** with EXACT content (copy of upstream `Predator.h` with namespace + class renamed and one new field):

```cpp
#ifndef C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_
#define C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_

#include <map>
#include <string>

#include "scrimmage/autonomy/Autonomy.h"

namespace c2overlay {
namespace autonomy {

class RemotePredator : public scrimmage::Autonomy {
 public:
  void init(std::map<std::string, std::string>& params) override;
  bool step_autonomy(double t, double dt) override;

 protected:
  // --- copied verbatim from upstream Predator.h ---
  int follow_id_;
  int prey_team_id_;
  double max_speed_;
  double capture_range_;
  bool allow_prey_switching_;
  scrimmage::PublisherPtr capture_ent_pub_;

  int speed_idx_ = 0;
  int turn_rate_idx_ = 0;
  int pitch_rate_idx_ = 0;

  int desired_heading_idx_ = 0;
  int desired_speed_idx_ = 0;

  // --- new for RemotePredator ---
  // -1 = no operator assignment, > 0 = locked target id.
  int assigned_target_id_ = -1;
};

}  // namespace autonomy
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_
```

- [ ] **Step 2: Create `RemotePredator.xml`** with EXACT content (copy of upstream `Predator.xml`, library name changed):

```xml
<?xml version="1.0"?>
<params>
  <library>RemotePredator_plugin</library>
  <max_speed>30</max_speed>
  <capture_range>5</capture_range>
  <allow_prey_switching>true</allow_prey_switching>
  <prey_team_id>1</prey_team_id>
</params>
```

- [ ] **Step 3: Create `RemotePredator.cpp`** with EXACT content (copy of upstream `src/plugins/autonomy/Predator/Predator.cpp` with namespace, class, REGISTER_PLUGIN renamed; new include for the command proto; new subscribe call in init; new override block at top of step_autonomy):

```cpp
#include "c2overlay/plugins/autonomy/RemotePredator/RemotePredator.h"

#include <limits>

#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/Angles.h"
#include "scrimmage/math/State.h"
#include "scrimmage/msgs/Capture.pb.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/pubsub/Message.h"
#include "scrimmage/pubsub/Publisher.h"

#include "Commands.pb.h"

namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Autonomy, c2overlay::autonomy::RemotePredator, RemotePredator_plugin)

namespace c2overlay {
namespace autonomy {

void RemotePredator::init(std::map<std::string, std::string>& params) {
  max_speed_ = scrimmage::get<double>("max_speed", params, 21);
  capture_range_ = scrimmage::get<double>("capture_range", params, 5);
  prey_team_id_ = scrimmage::get<int>("prey_team_id", params, 1);

  allow_prey_switching_ = scrimmage::get<bool>("allow_prey_switching", params, false);

  capture_ent_pub_ = advertise("GlobalNetwork", "CaptureEntity");

  follow_id_ = -1;

  speed_idx_ = vars_.declare(scrimmage::VariableIO::Type::speed, scrimmage::VariableIO::Direction::Out);
  turn_rate_idx_ = vars_.declare(scrimmage::VariableIO::Type::turn_rate, scrimmage::VariableIO::Direction::Out);
  pitch_rate_idx_ = vars_.declare(scrimmage::VariableIO::Type::pitch_rate, scrimmage::VariableIO::Direction::Out);

  desired_heading_idx_ =
      vars_.declare(scrimmage::VariableIO::Type::desired_heading, scrimmage::VariableIO::Direction::Out);
  desired_speed_idx_ = vars_.declare(scrimmage::VariableIO::Type::desired_speed, scrimmage::VariableIO::Direction::Out);

  // --- NEW: subscribe to operator commands. ---
  auto cb = [this](scrimmage::MessagePtr<c2overlay_msgs::TargetAssignment> msg) {
    if (msg->data.predator_id() != parent_->id().id()) return;  // not for me
    if (msg->data.target_id() == 0) {
      assigned_target_id_ = -1;  // clear
    } else {
      assigned_target_id_ = msg->data.target_id();
    }
  };
  subscribe<c2overlay_msgs::TargetAssignment>("GlobalNetwork", "Commands/TargetAssignment", cb);
}

bool RemotePredator::step_autonomy(double t, double dt) {
  // --- NEW: hard-lock assigned target if valid; else fall through to upstream auto-pick. ---
  bool assignment_active = false;
  if (assigned_target_id_ > 0) {
    if (contacts_->count(assigned_target_id_) > 0) {
      follow_id_ = assigned_target_id_;
      assignment_active = true;
    } else {
      // Assigned target is gone — clear and let auto-pick resume.
      assigned_target_id_ = -1;
    }
  }

  // --- AUTO-PICK BLOCK (upstream behavior) ---
  // Only runs when no operator assignment is active, OR if follow_id_ went stale.
  if (!assignment_active) {
    if (contacts_->count(follow_id_) == 0) {
      follow_id_ = -1;
    }

    if (follow_id_ < 0 || allow_prey_switching_) {
      double min_dist = std::numeric_limits<double>::infinity();
      for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
        if (it->second.id().team_id() != prey_team_id_)
          continue;
        double dist = (it->second.state()->pos() - state_->pos()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          follow_id_ = it->first;
        }
      }
    }
  }

  // --- CAPTURE PUBLISHING (upstream behavior, unchanged) ---
  for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
    if (it->second.id().team_id() == parent_->id().team_id())
      continue;
    double dist = (it->second.state()->pos() - state_->pos()).norm();
    if (dist < capture_range_) {
      auto msg = std::make_shared<scrimmage::Message<sm::CaptureEntity>>();
      msg->data.set_source_id(parent_->id().id());
      msg->data.set_target_id(it->second.id().id());
      capture_ent_pub_->publish(msg);
    }
  }

  // --- CHASE LOGIC (upstream behavior, unchanged) ---
  if (contacts_->count(follow_id_) > 0) {
    scrimmage::StatePtr ent_state = contacts_->at(follow_id_).state();
    Eigen::Vector3d v = (ent_state->pos() - state_->pos()).normalized() * max_speed_;
    double desired_heading = std::atan2(v(1), v(0));
    double desired_pitch = std::atan2(v(2), v.head<2>().norm());

    vars_.output(speed_idx_, max_speed_);
    vars_.output(turn_rate_idx_, scrimmage::Angles::angle_pi(desired_heading - state_->quat().yaw()));
    vars_.output(pitch_rate_idx_, scrimmage::Angles::angle_pi(desired_pitch + state_->quat().pitch()));

    vars_.output(desired_heading_idx_, desired_heading);
    vars_.output(desired_speed_idx_, max_speed_);
  }

  return true;
}

}  // namespace autonomy
}  // namespace c2overlay
```

- [ ] **Step 4: Create `webapp/scrimmage-overlay/src/plugins/autonomy/RemotePredator/CMakeLists.txt`** with EXACT content:

```cmake
set(LIBRARY_NAME RemotePredator_plugin)

set(SRCS
  RemotePredator.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRCS})

target_link_libraries(${LIBRARY_NAME}
  c2overlay-msgs
  scrimmage-core
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
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/include/c2overlay/plugins/autonomy/ webapp/scrimmage-overlay/src/plugins/autonomy/
git -C C:/Git/kinetas/scrimmage commit -m "feat(autonomy): RemotePredator — Predator + operator-controllable target lock"
```

---

## Task 5: Wire RemotePredator into the overlay top-level CMakeLists

**Files:**
- Modify: `webapp/scrimmage-overlay/CMakeLists.txt`

- [ ] **Step 1: Edit `webapp/scrimmage-overlay/CMakeLists.txt`**

Find the line `add_subdirectory(src/plugins/interaction/TopicTap)`. Add immediately after it:

```cmake
add_subdirectory(src/plugins/autonomy/RemotePredator)
```

- [ ] **Step 2: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/CMakeLists.txt
git -C C:/Git/kinetas/scrimmage commit -m "build(overlay): add RemotePredator subdirectory to top-level CMakeLists"
```

---

## Task 6: Mission XML — swap Predator → RemotePredator

**Files:**
- Modify: `webapp/scrimmage-overlay/missions/predator_prey_boids.xml`

- [ ] **Step 1: Edit `webapp/scrimmage-overlay/missions/predator_prey_boids.xml`**

Find the predator entity block (the one with `team_id=2`, near the bottom). The line:

```xml
    <autonomy allow_prey_switching="true" capture_range="5" max_speed="35">Predator</autonomy>
```

Change `Predator` to `RemotePredator`:

```xml
    <autonomy allow_prey_switching="true" capture_range="5" max_speed="35">RemotePredator</autonomy>
```

Nothing else changes — `RemotePredator.xml` accepts the same parameters as upstream `Predator.xml`. The `<entity_interaction>TopicTap</entity_interaction>` block stays untouched (kill feed still works).

- [ ] **Step 2: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/missions/predator_prey_boids.xml
git -C C:/Git/kinetas/scrimmage commit -m "feat(missions): predator_prey_boids uses RemotePredator (operator-controllable)"
```

---

## Task 7: API — `PublishToTopicAsync` helper + 2 endpoints

**Files:**
- Modify: `webapp/api/TopicTapClient.cs`
- Modify: `webapp/api/Endpoints.cs`

- [ ] **Step 1: Edit `webapp/api/TopicTapClient.cs` — expose a public `PublishToTopicAsync` helper**

Read the current file to see the namespace (`C2.Api`) and existing using imports. Then add a new public method on the `TopicTapClient` class. A good place is right after the `ExecuteAsync` method body.

```csharp
public async Task<(bool ok, string error)> PublishToTopicAsync(
    string network,
    string topic,
    string payloadJson,
    CancellationToken ct = default)
{
    using var channel = GrpcChannel.ForAddress(_addr);
    var client = new TopicTapService.TopicTapServiceClient(channel);
    var req = new PublishToTopicRequest
    {
        Network = network,
        Topic = topic,
        PayloadJson = payloadJson,
    };
    var resp = await client.PublishToTopicAsync(req, cancellationToken: ct);
    return (resp.Ok, resp.Error);
}
```

Note: `_addr` is the existing private field set from the `TOPICTAP_GRPC_ADDR` config; reuse it. Channel is created per call (cheap; consistent with the existing `RunOnce` pattern). RpcException propagates to the caller — the endpoint translates it.

- [ ] **Step 2: Edit `webapp/api/Endpoints.cs` — add the two new POST endpoints**

Inside the `MapMissionEndpoints` extension method (or wherever the existing `MapGet("/api/topics", ...)` lives), add the following endpoints. Both call the same `TopicTapClient.PublishToTopicAsync`.

Generic publish:

```csharp
app.MapPost("/api/topics/{network}/{topic}/publish",
    async (string network, string topic, GenericPublishRequest body, TopicTapClient client) =>
    {
        topic = Uri.UnescapeDataString(topic);
        try
        {
            var (ok, error) = await client.PublishToTopicAsync(network, topic, body.PayloadJson);
            if (!ok) return Results.BadRequest(new { ok = false, error });
            return Results.Ok(new { ok = true });
        }
        catch (Grpc.Core.RpcException ex)
        {
            return Results.Json(
                new { ok = false, error = $"TopicTap unavailable: {ex.Status.Detail}" },
                statusCode: 502);
        }
    });
```

Typed sugar:

```csharp
app.MapPost("/api/commands/target-assignment",
    async (TargetAssignmentRequest body, TopicTapClient client) =>
    {
        if (body.PredatorId <= 0)
            return Results.BadRequest(new { ok = false, error = "predatorId must be > 0" });
        if (body.TargetId < 0)
            return Results.BadRequest(new { ok = false, error = "targetId must be >= 0" });

        var payloadJson = System.Text.Json.JsonSerializer.Serialize(new
        {
            predatorId = body.PredatorId,
            targetId = body.TargetId,
        });

        try
        {
            var (ok, error) = await client.PublishToTopicAsync(
                "GlobalNetwork", "Commands/TargetAssignment", payloadJson);
            if (!ok) return Results.BadRequest(new { ok = false, error });
            return Results.Ok(new { ok = true });
        }
        catch (Grpc.Core.RpcException ex)
        {
            return Results.Json(
                new { ok = false, error = $"TopicTap unavailable: {ex.Status.Detail}" },
                statusCode: 502);
        }
    });
```

Add the two record DTOs at the end of `Endpoints.cs` (inside the `C2.Api` namespace, outside the `Endpoints` static class):

```csharp
public sealed record GenericPublishRequest(string PayloadJson);

public sealed record TargetAssignmentRequest(int PredatorId, int TargetId);
```

If `Endpoints.cs` doesn't already have these usings, add them at the top:

```csharp
using ScrimmageC2.Generated.TopicTap;
```

(For the `TopicTapClient` reference; it's already injected via DI from the existing TopicTap wiring.)

- [ ] **Step 3: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/api/TopicTapClient.cs webapp/api/Endpoints.cs
git -C C:/Git/kinetas/scrimmage commit -m "feat(api): PublishToTopicAsync helper + generic /api/topics publish + typed /api/commands/target-assignment"
```

---

## Task 8: Web — `commandsApi.ts`

**Files:**
- Create: `webapp/web/src/lib/commandsApi.ts`

- [ ] **Step 1: Create `webapp/web/src/lib/commandsApi.ts`**

```typescript
const API_URL = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

export interface CommandResult {
  ok: boolean;
  error?: string;
}

async function postJson(path: string, body: unknown): Promise<CommandResult> {
  const res = await fetch(`${API_URL}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  let parsed: CommandResult | null = null;
  try { parsed = await res.json() as CommandResult; } catch { /* no-op */ }
  if (!res.ok) {
    return { ok: false, error: parsed?.error ?? `HTTP ${res.status}` };
  }
  return parsed ?? { ok: false, error: 'empty response' };
}

export function assignTarget(predatorId: number, targetId: number): Promise<CommandResult> {
  return postJson('/api/commands/target-assignment', { predatorId, targetId });
}

export function clearTarget(predatorId: number): Promise<CommandResult> {
  return postJson('/api/commands/target-assignment', { predatorId, targetId: 0 });
}
```

- [ ] **Step 2: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/web/src/lib/commandsApi.ts
git -C C:/Git/kinetas/scrimmage commit -m "feat(web): commandsApi — assignTarget / clearTarget POST helpers"
```

---

## Task 9: Web — extend `EntityInspectorCard` with assignment button

**Files:**
- Modify: `webapp/web/src/components/EntityInspectorCard.tsx`

- [ ] **Step 1: Read the current `EntityInspectorCard.tsx`** to understand its props, layout conventions, and what frame data it already has access to. The user has been actively developing this — match existing patterns. Specifically note:
  - How it gets the latest frame (probably via a prop or hook).
  - Class/styling conventions (per Task 14 of TopicTap plan, the codebase uses inline styles with CSS vars like `var(--accent)`, `var(--text-primary)`, etc.).
  - Whether it currently has any imperative side-effects (button handlers).

- [ ] **Step 2: Add imports at the top of `EntityInspectorCard.tsx`**

```tsx
import { useEffect, useState } from 'react';   // useState may already be imported; merge
import { assignTarget, clearTarget } from '../lib/commandsApi';
```

- [ ] **Step 3: Inside the component, derive `predatorEntityId` from the latest frame**

Use whatever access pattern the component already has to the latest frame. If it gets frames via a `latestFrame` prop or via the `useFrameStream` hook, reuse that. Add this derivation near the top of the component body:

```tsx
// Pick the first team-2 entity as "the predator." v1 limitation: multi-predator UX deferred.
const predatorEntityId = latestFrame?.entities.find(e => e.teamId === 2)?.id ?? null;
```

(Replace `latestFrame` with whatever the actual variable is in the file. If `useFrameStream` exposes it differently, adapt.)

- [ ] **Step 4: Add local state for the active assignment + command error**

Inside the component:

```tsx
const [assignedTargetId, setAssignedTargetId] = useState<number | null>(null);
const [commandError, setCommandError] = useState<string | null>(null);

// Auto-clear error after 3s.
useEffect(() => {
  if (commandError == null) return;
  const t = window.setTimeout(() => setCommandError(null), 3000);
  return () => window.clearTimeout(t);
}, [commandError]);

// If the assigned target leaves the frame (e.g., captured), drop the local lock.
useEffect(() => {
  if (assignedTargetId == null || latestFrame == null) return;
  const stillAlive = latestFrame.entities.some(e => e.id === assignedTargetId);
  if (!stillAlive) setAssignedTargetId(null);
}, [latestFrame, assignedTargetId]);
```

- [ ] **Step 5: Render the assignment section conditionally**

Inside the JSX, ONLY when (a) the inspector is in `live` mode, (b) the entity's `teamId === 1` (prey), and (c) `predatorEntityId != null`. Find a sensible spot in the card layout (e.g., below the entity stats, above the "captured" placeholder area).

```tsx
{state.mode === 'live'
  && state.entity != null
  && state.entity.teamId === 1
  && predatorEntityId != null && (
  <div style={{ padding: '12px 14px', borderTop: '1px solid var(--border-default)' }}>
    {assignedTargetId === state.entity.id ? (
      <button
        type="button"
        onClick={async () => {
          const r = await clearTarget(predatorEntityId);
          if (r.ok) setAssignedTargetId(null);
          else setCommandError(r.error ?? 'unknown error');
        }}
        style={{
          width: '100%', padding: '8px 12px', cursor: 'pointer',
          background: 'var(--accent)', color: 'var(--bg-base)',
          border: 'none', fontFamily: 'var(--font-mono)', fontSize: '12px',
          fontWeight: 600, textTransform: 'uppercase', letterSpacing: '0.05em',
        }}>
        Clear predator target
      </button>
    ) : (
      <button
        type="button"
        onClick={async () => {
          const id = state.entity!.id;
          const r = await assignTarget(predatorEntityId, id);
          if (r.ok) setAssignedTargetId(id);
          else setCommandError(r.error ?? 'unknown error');
        }}
        style={{
          width: '100%', padding: '8px 12px', cursor: 'pointer',
          background: 'transparent', color: 'var(--accent)',
          border: '1px solid var(--accent)', fontFamily: 'var(--font-mono)',
          fontSize: '12px', fontWeight: 600,
          textTransform: 'uppercase', letterSpacing: '0.05em',
        }}>
        Set as predator target
      </button>
    )}
    {commandError && (
      <div style={{
        marginTop: '6px',
        color: 'var(--danger, #c44)',
        fontFamily: 'var(--font-mono)',
        fontSize: '11px',
      }}>{commandError}</div>
    )}
  </div>
)}
```

(If the existing card uses different style conventions, adapt — the behavior is what matters: a button that toggles label between "Set" / "Clear" and shows inline errors.)

- [ ] **Step 6: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/web/src/components/EntityInspectorCard.tsx
git -C C:/Git/kinetas/scrimmage commit -m "feat(web): EntityInspectorCard — Set/Clear predator target button"
```

---

## Task 10: C++ unit test for publish dispatch

**Files:**
- Create: `webapp/scrimmage-overlay/test/test_target_assignment_dispatch.cpp`
- Modify: `webapp/scrimmage-overlay/test/CMakeLists.txt`

- [ ] **Step 1: Create `webapp/scrimmage-overlay/test/test_target_assignment_dispatch.cpp`**

```cpp
#include <cassert>
#include <iostream>
#include <string>

#include <google/protobuf/util/json_util.h>
#include "Commands.pb.h"

int main() {
  // Round-trip: web sends camelCase JSON; protobuf parser must accept it.
  const std::string in = "{\"predatorId\":101,\"targetId\":47}";

  c2overlay_msgs::TargetAssignment msg;
  google::protobuf::util::JsonParseOptions opts;
  opts.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(in, &msg, opts);
  if (!status.ok()) {
    std::cerr << "parse failed: " << status.message() << "\n";
    return 1;
  }
  if (msg.predator_id() != 101) {
    std::cerr << "predator_id mismatch: got " << msg.predator_id() << "\n";
    return 1;
  }
  if (msg.target_id() != 47) {
    std::cerr << "target_id mismatch: got " << msg.target_id() << "\n";
    return 1;
  }

  // Sentinel clear: target_id == 0
  c2overlay_msgs::TargetAssignment clr;
  auto s2 = google::protobuf::util::JsonStringToMessage(
      "{\"predatorId\":101,\"targetId\":0}", &clr, opts);
  if (!s2.ok() || clr.target_id() != 0) {
    std::cerr << "clear sentinel test failed\n";
    return 1;
  }

  std::cout << "OK\n";
  return 0;
}
```

- [ ] **Step 2: Edit `webapp/scrimmage-overlay/test/CMakeLists.txt`** to add the new test target

Append to the file:

```cmake
add_executable(test_target_assignment_dispatch test_target_assignment_dispatch.cpp)
# Same link list as test_capture_entity_json — c2overlay-msgs brings the typed
# protobuf, plus core/grpc to satisfy transitive deps.
target_link_libraries(test_target_assignment_dispatch
  c2overlay-msgs
  scrimmage-core
  protobuf::libprotobuf
  gRPC::grpc++
)
add_test(NAME test_target_assignment_dispatch COMMAND test_target_assignment_dispatch)
```

- [ ] **Step 3: Commit**

```bash
git -C C:/Git/kinetas/scrimmage add webapp/scrimmage-overlay/test/
git -C C:/Git/kinetas/scrimmage commit -m "test(commands): TargetAssignment JSON parse round-trip (camelCase + sentinel)"
```

---

## Task 11: Manual smoke test (acceptance)

This is the only test that absolutely must pass before declaring v1 done. Per the spec.

- [ ] **Step 1: Clean rebuild**

```bash
docker compose -f webapp/docker-compose.yml down -v
docker compose -f webapp/docker-compose.yml up --build
```

Expected: scrimmage container builds the overlay (now including `RemotePredator_plugin.so`); api container builds with the new proto + endpoints; web container starts.

- [ ] **Step 2: Verify the C++ unit test passes inside the container** (one-time check after the first successful build)

```bash
docker compose -f webapp/docker-compose.yml exec scrimmage \
  bash -c 'cd /opt/c2overlay/build && ctest --output-on-failure'
```

Expected: 2/2 tests pass (test_capture_entity_json from TopicTap v1, test_target_assignment_dispatch new in this iteration).

- [ ] **Step 3: Open browser** → `http://localhost:5173`.

- [ ] **Step 4: Start `predator_prey_boids.xml`**

Wait for the scene to render. Confirm:
- 100 blue boids, 1 red predator visible in Cesium.
- Predator auto-picks nearest blue (visible movement).
- Right-rail topic feed eventually shows `CaptureEntity` events as the predator catches prey (regression check on TopicTap v1).

- [ ] **Step 5: Click any blue boid in Cesium**

EntityInspectorCard appears with the boid's stats. Mode is `live`. The "Set as predator target" button appears below the stats.

- [ ] **Step 6: Click "Set as predator target"**

Within ~1 second, the predator visibly turns toward the selected boid and chases it (ignoring closer alternatives). Button label flips to "Clear predator target."

- [ ] **Step 7: Pick a different blue boid** (the predator is currently chasing the first one)

Click the new boid in Cesium → inspector switches to it → "Set as predator target" shows again (since `assignedTargetId` is the OLD boid's id, not this one). Click → predator switches to the new target.

- [ ] **Step 8: Click "Clear predator target"**

Predator visibly resumes auto-pick (nearest blue boid). Button reverts to "Set as predator target" (the cleared assignment is reflected in local state).

- [ ] **Step 9: Re-assign, then wait for the catch**

Pick a blue boid → "Set as predator target" → wait for predator to reach it. Captured boid disappears from Cesium. EntityInspectorCard enters `captured` mode for ~2s. Predator visibly auto-picks the next-nearest blue. Local `assignedTargetId` clears (the useEffect detects target left frame).

- [ ] **Step 10: Tag the release**

```bash
git -C C:/Git/kinetas/scrimmage tag target-assignment-v1
```

(No commit needed — all work was committed in earlier tasks.)

---

## Self-review checklist

After all tasks complete, verify:

- [ ] `git -C C:/Git/kinetas/scrimmage log --oneline | head -15` shows ~10 commits with `feat(...)` / `test(...)` / `build(...)` prefixes.
- [ ] Spec §4.1 (`TopicTap` plugin extension) — Task 3.
- [ ] Spec §4.2 (`Commands.proto`) — Task 1.
- [ ] Spec §4.3 (`RemotePredator`) — Task 4 + Task 5 + Task 6.
- [ ] Spec §4.4 (API endpoints) — Task 7.
- [ ] Spec §4.5 (`EntityInspectorCard` extension) — Task 9.
- [ ] Spec §4.6 (mission XML) — Task 6.
- [ ] Spec §5 (wire formats) — Tasks 1, 2, 7.
- [ ] Spec §7 (error handling) — implemented in Task 7 (API validation, gRPC exception translation), Task 3 (publish_registry unknown-topic, JSON parse), Task 4 (RemotePredator silent ignore on invalid id).
- [ ] Spec §8.2 (one C++ unit test) — Task 10.
- [ ] Spec §8.1 (manual smoke) — Task 11.

If any spec section is uncovered, add a task before declaring complete.
