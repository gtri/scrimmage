# Target Assignment Command — Design (v1)

**Date:** 2026-05-02
**Author:** Scott McCutchen
**Status:** Approved, ready for implementation planning
**Context:** Hackathon push on top of the SCRIMMAGE C2 UI MVP + TopicTap v1 (`2026-05-02-pubsub-bridge-design.md`). Solo developer.

## 1. Overview

The first operator command in the C2 UI: **target assignment**. Operator selects a prey boid in the EntityInspectorCard and clicks "Set as predator target." The predator overrides its automatic nearest-enemy selection and chases the chosen entity until the operator clears the assignment or the assigned target is destroyed.

This iteration also lights up the **inject side** of TopicTap (the v2 deferred from the pub/sub bridge work) by adding a generic `PublishToTopic` gRPC + REST surface. The typed `target-assignment` endpoint is sugar over that generic surface; future commands (swap_team, set_speed, etc.) reuse the same plumbing without API or proto changes.

### In scope (v1)

- New `RemotePredator` overlay autonomy plugin — copy-and-modify of upstream `Predator` that subscribes to `Commands/TargetAssignment` and overrides target selection.
- Extension to `TopicTap` plugin — adds `PublishToTopic(network, topic, payload_json)` unary gRPC RPC backed by a static publish_registry mapping `(network, topic) → typed handler`.
- New protobuf message `c2overlay_msgs::TargetAssignment {predator_id, target_id}` in a new `Commands.proto`.
- Two new API endpoints — generic `POST /api/topics/{network}/{topic}/publish` and typed sugar `POST /api/commands/target-assignment`.
- New "Set / Clear predator target" button on the existing `EntityInspectorCard` (only shown when selected entity is on team 1 = prey).
- Mission XML edit — overlay's `predator_prey_boids.xml` switches the predator from `Predator` to `RemotePredator`.

### Out of scope (deferred)

- **Generic "publish to a topic" UI** — the editor + topic dropdown that exercises the generic API endpoint by hand. Plumbing is built in v1; UI is deferred.
- **Multi-predator UI** — when more than one team-2 entity exists. Wire format already accommodates; UI v1 sends to "first predator found."
- **Audit log** to Postgres for issued commands.
- **Other commands** (swap_team, set_speed, recall_to_base, etc.).
- **Confirmation modal** before issuing assignment.
- **Toast notifications** for success — inline error under the button is the only feedback.
- **Synced UI assignment state** across browser refresh / multiple tabs.
- **Querying current assignment from the sim** — web only knows what it last sent.

## 2. Locked-in design decisions

| # | Decision | Rationale |
|---|---|---|
| 1 | Hackathon scope, aggressive YAGNI | Same constraint as TopicTap v1 |
| 2 | **Hard lock** override; **revert to auto-pick on death** | Most demoable; matches operator-takes-control story; trivial to implement |
| 3 | Generic `PublishToTopic` gRPC + generic REST endpoint + typed REST sugar; UI = single button (no JSON editor) | Generic plumbing keeps later commands cheap; typed UI keeps demo focused |
| 4 | Broadcast topic `Commands/TargetAssignment` + sentinel `target_id=0` for clear | Matches existing SCRIMMAGE pub/sub patterns; one proto, one topic, one button |
| 5 | New **`RemotePredator`** overlay autonomy plugin (no upstream touches) | Fork stays clean; same overlay-as-quarantine pattern as TopicTap |

## 3. Architecture (delta vs current state)

```
                          ┌─────────────────────────────────────┐
                          │  EntityInspectorCard (web)          │
                          │  - selected boid id                 │
                          │  - "Set as predator target" button  │
                          │  - "Clear" button when assigned     │
                          └──────────────┬──────────────────────┘
                                         │ POST /api/commands/target-assignment
                                         │ {predatorId, targetId}
                                         ▼
                          ┌─────────────────────────────────────┐
                          │  api (.NET)                         │
                          │  - typed endpoint (sugar)           │
                          │  - generic endpoint POST /api/      │
                          │    topics/{net}/{topic}/publish     │
                          │  - both call PublishToTopic gRPC    │
                          └──────────────┬──────────────────────┘
                                         │ gRPC PublishToTopic
                                         │ ★ NEW unary RPC on TopicTapService
                                         ▼
                          ┌─────────────────────────────────────┐
                          │  TopicTap plugin (sim) [extended]   │
                          │  - existing: ListTopics+StreamTopic │
                          │  - ★ NEW: PublishToTopic dispatches │
                          │    to a static publish_registry     │
                          │    keyed on (network, topic) →      │
                          │    typed publish handler            │
                          └──────────────┬──────────────────────┘
                                         │ scrimmage::Publisher::publish()
                                         │ on GlobalNetwork/TargetAssignment
                                         ▼
                          ┌─────────────────────────────────────┐
                          │  RemotePredator (sim, NEW autonomy) │
                          │  - subscribes Commands/TargetAssign │
                          │  - filters by predator_id == own_id │
                          │  - target_id > 0 → hard lock        │
                          │  - target_id == 0 → clear lock      │
                          │  - target dies → auto-pick resumes  │
                          └─────────────────────────────────────┘
```

Five new pieces:
- New unary gRPC RPC `PublishToTopic` extending `TopicTapService`
- New API endpoints (1 generic + 1 typed sugar)
- New autonomy plugin `RemotePredator` in the overlay
- New proto for the command message
- New UI button on `EntityInspectorCard`

## 4. Components

### 4.1 `TopicTap` plugin extension (C++ overlay)

**Files modified:**
- `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTap.h`
- `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTap.cpp`
- `webapp/scrimmage-overlay/include/c2overlay/plugins/interaction/TopicTap/TopicTapServiceImpl.h`
- `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/TopicTapServiceImpl.cpp`
- `webapp/scrimmage-overlay/src/plugins/interaction/TopicTap/CMakeLists.txt` (link Commands.proto generated lib)

**Threading model:** mirrors upstream `GRPCCommandString`. The gRPC server thread does NOT publish directly onto the SCRIMMAGE bus (pub/sub is single-threaded by convention). Instead the gRPC handler validates + parses on the gRPC thread, enqueues a typed message into a thread-safe FIFO, and returns. The next `step_entity_interaction` call (sim thread) drains the queue and calls `Publisher::publish()` from there.

**New state on `TopicTap`:**
- Static `publish_registry` mapping `(network, topic) → handler` where each handler validates the JSON shape and produces a type-erased `PendingPublish` (carries the parsed message pointer + a closure that knows how to publish it from the sim thread).
- `std::deque<PendingPublish> pending_publishes_` + `std::mutex pending_publishes_mutex_` — the FIFO between gRPC and sim threads.
- `std::map<string, scrimmage::PublisherPtr> pub_cache_` keyed on `network:topic` — populated lazily by the sim-thread drain code. Only ever touched from the sim thread, no lock needed.

**gRPC handler flow** (`TopicTapServiceImpl::PublishToTopic`):
1. Look up `(req.network(), req.topic())` in `publish_registry`. Unknown → return `PublishToTopicResponse{ok: false, error: "unknown publish topic '...'"}`.
2. Invoke the handler on the request's `payload_json`. Handler calls `google::protobuf::util::JsonStringToMessage(...)`. On parse error → `PublishToTopicResponse{ok: false, error: "json parse: ..."}`.
3. On success → handler returns a `PendingPublish` carrying the typed `MessagePtr<T>` and the cached-or-create `(network, topic)` key. gRPC handler enqueues onto `pending_publishes_` under the mutex.
4. Returns `PublishToTopicResponse{ok: true}` — note this is a "queued for publish" ack, not a confirmation that the message has been delivered to subscribers. Sim thread will publish it on the next tick (≤ 100ms latency at default 10 Hz sim rate).

**Sim-thread drain** (`step_entity_interaction`, currently a no-op for the observe path):
1. Lock `pending_publishes_mutex_`, swap the deque into a local, unlock.
2. For each `PendingPublish`: lazily `advertise(network, topic)` if not in `pub_cache_`, then call `publisher->publish(typed_message)`.

This preserves the single-threaded pub/sub assumption that SCRIMMAGE's pub/sub layer relies on, and matches the proven pattern in `src/plugins/interaction/GRPCCommandString/GRPCCommandString.cpp:72-103`.

**Adding a new publishable topic:** one line in `publish_registry`, rebuild. Same recompile cost as the observe-side `type_registry`.

### 4.2 `Commands.proto` (overlay)

**File:** `webapp/scrimmage-overlay/msgs/Commands.proto` (new)

```proto
syntax = "proto3";

package c2overlay_msgs;

option csharp_namespace = "ScrimmageC2.Generated.Commands";

message TargetAssignment {
  int32 predator_id = 1;       // entity id of the predator to command; > 0
  int32 target_id = 2;          // entity id of the target; 0 = clear assignment
}
```

Compiled into the existing `c2overlay-msgs` static library via `webapp/scrimmage-overlay/msgs/CMakeLists.txt` edit (add `Commands.proto` to `PROTO_SRCS`).

Mirrored at `webapp/api/Protos/scrimmage/Commands.proto` for C# generation; registered in `Api.csproj`.

### 4.3 `RemotePredator` overlay autonomy (C++)

**Files (new):**
- `webapp/scrimmage-overlay/include/c2overlay/plugins/autonomy/RemotePredator/RemotePredator.h`
- `webapp/scrimmage-overlay/include/c2overlay/plugins/autonomy/RemotePredator/RemotePredator.xml`
- `webapp/scrimmage-overlay/src/plugins/autonomy/RemotePredator/RemotePredator.cpp`
- `webapp/scrimmage-overlay/src/plugins/autonomy/RemotePredator/CMakeLists.txt`

**Source:** copy of upstream `src/plugins/autonomy/Predator/Predator.cpp` (~140 lines) renamed and modified.

**Additional state:**
```cpp
int assigned_target_id_ = -1;  // -1 = no assignment, >0 = locked target
```

**Init additions:**
```cpp
auto cb = [this](scrimmage::MessagePtr<c2overlay_msgs::TargetAssignment> msg) {
  if (msg->data.predator_id() != parent_->id().id()) return;  // not for me
  if (msg->data.target_id() == 0) {
    assigned_target_id_ = -1;  // clear
  } else {
    assigned_target_id_ = msg->data.target_id();
  }
};
subscribe<c2overlay_msgs::TargetAssignment>("GlobalNetwork", "Commands/TargetAssignment", cb);
```

**`step_autonomy` modifications:** at the top, before the existing nearest-enemy logic:
```cpp
if (assigned_target_id_ > 0) {
  if (contacts_->count(assigned_target_id_) > 0) {
    follow_id_ = assigned_target_id_;
    // skip the auto-switching block by jumping straight to the chase logic
    goto chase;  // or refactor to extract chase into a helper
  } else {
    // assigned target gone — clear and let auto-pick resume
    assigned_target_id_ = -1;
  }
}
// ... existing auto-switching block runs only when no valid assignment ...
chase:
// ... existing chase / capture / output logic, unchanged ...
```

(Implementation may use early-return + helper extraction instead of `goto` for style; semantics identical.)

**XML config:** RemotePredator.xml accepts the same parameters as upstream Predator (`max_speed`, `capture_range`, `prey_team_id`, `allow_prey_switching`). The `allow_prey_switching` flag still governs auto-pick behavior when no assignment is active — it has no effect during a hard lock.

### 4.4 API endpoints (.NET 10)

**Files modified:**
- `webapp/api/Endpoints.cs` (+ 2 endpoints)
- `webapp/api/TopicTapClient.cs` (+ public `PublishToTopicAsync` helper that wraps the generated gRPC client)
- `webapp/api/Api.csproj` (+ `Commands.proto`)
- `webapp/api/Protos/scrimmage/Commands.proto` (new mirror)
- `webapp/api/Protos/scrimmage/TopicTap.proto` (+ PublishToTopic RPC)

**Endpoint 1 — generic publish:**
```
POST /api/topics/{network}/{topic}/publish
Content-Type: application/json
Body:  { "payloadJson": "..." }      # serialized protobuf JSON for the topic's typed message

→ 200  { "ok": true }
→ 400  { "ok": false, "error": "json parse: ..." }
→ 502  { "ok": false, "error": "TopicTap unavailable: ..." }
```

`{topic}` is URL-encoded; the `/` in `Commands/TargetAssignment` becomes `%2F`. Implementation calls `TopicTapClient.PublishToTopicAsync(network, topic, payloadJson)`.

**Endpoint 2 — typed sugar:**
```
POST /api/commands/target-assignment
Content-Type: application/json
Body:  { "predatorId": 101, "targetId": 47 }

→ 200  { "ok": true }
→ 400  { "ok": false, "error": "predatorId must be > 0" }
→ 502  { "ok": false, "error": "..." }
```

Validates `predatorId > 0` and `targetId >= 0`. Constructs `payloadJson = JsonSerializer.Serialize(new { predatorId, targetId })` and calls the same `PublishToTopicAsync` path. The plugin's protobuf JSON parser accepts that field-name casing because the generated C# classes use the same camelCase.

**`TopicTapClient.PublishToTopicAsync(network, topic, payloadJson)`:** uses the existing `_addr` and a fresh `GrpcChannel` per call (channels are cheap; this is not on the hot frame path). Wraps the unary RPC and returns the response. Errors propagate as `RpcException` for the endpoint handler to translate.

### 4.5 Web — `EntityInspectorCard` extension

**Files modified:**
- `webapp/web/src/components/EntityInspectorCard.tsx`

**Files added:**
- `webapp/web/src/lib/commandsApi.ts` — thin POST helpers: `assignTarget(predatorId, targetId)` and `clearTarget(predatorId)`.

**State additions to the card (or a sibling hook):**
- `predatorEntityId: number | null` — derived from the latest frame stream (first entity with `teamId === 2`).
- `assignedTargetId: number | null` — locally tracked; updated optimistically on successful 200 response.
- `commandError: string | null` — populated on 4xx/5xx; auto-clears after 3s.

**Render rules:**
- Only show the assignment section when `mode === 'live'`, `entity.teamId === 1` (prey), and `predatorEntityId != null`.
- Button label:
  - `assignedTargetId === entity.id` → "Clear predator target"
  - else → "Set as predator target"
- `commandError` rendered inline under the button when non-null.

**Click handler:**
- "Set" → `assignTarget(predatorEntityId, entity.id)`. On 200 → set `assignedTargetId = entity.id`. On error → set `commandError`.
- "Clear" → `clearTarget(predatorEntityId)` (sends `targetId: 0`). On 200 → set `assignedTargetId = null`.

### 4.6 Mission XML edit

**File modified:** `webapp/scrimmage-overlay/missions/predator_prey_boids.xml`

```xml
<!-- before -->
<entity entity_common="predator">
  <team_id>2</team_id>
  <count>1</count>
  <color>255 0 0</color>
  <autonomy allow_prey_switching="true" capture_range="5" max_speed="35">Predator</autonomy>
</entity>

<!-- after -->
<entity entity_common="predator">
  <team_id>2</team_id>
  <count>1</count>
  <color>255 0 0</color>
  <autonomy allow_prey_switching="true" capture_range="5" max_speed="35">RemotePredator</autonomy>
</entity>
```

The TopicTap `<entity_interaction>` block is untouched. Existing kill feed (CaptureEntity events) continues to work because the RemotePredator publishes the same `CaptureEntity` message on the same topic as the upstream Predator (that publish call is preserved verbatim in the copy).

## 5. Wire formats

### 5.1 SCRIMMAGE → API (extending TopicTap.proto)

```proto
service TopicTapService {
  rpc ListTopics(ListTopicsRequest) returns (TopicList);
  rpc StreamTopic(StreamTopicRequest) returns (stream TopicMessage);
  rpc PublishToTopic(PublishToTopicRequest) returns (PublishToTopicResponse);  // NEW
}

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

`PublishToTopicResponse` is a structured ok+error pair (not `Empty`) so the plugin can return parse / unknown-topic errors as data rather than gRPC status codes.

### 5.2 New `Commands.proto`

```proto
// webapp/scrimmage-overlay/msgs/Commands.proto
syntax = "proto3";
package c2overlay_msgs;
option csharp_namespace = "ScrimmageC2.Generated.Commands";

message TargetAssignment {
  int32 predator_id = 1;       // > 0
  int32 target_id = 2;          // 0 = clear, > 0 = assign
}
```

Mirrored at `webapp/api/Protos/scrimmage/Commands.proto` for C# generation.

### 5.3 API → web (REST)

Typed sugar:
```
POST /api/commands/target-assignment
Body:  { "predatorId": 101, "targetId": 47 }
→ 200  { "ok": true }
→ 400  { "ok": false, "error": "predatorId must be > 0" }
→ 502  { "ok": false, "error": "TopicTap unavailable: <reason>" }
```

Generic:
```
POST /api/topics/GlobalNetwork/Commands%2FTargetAssignment/publish
Body:  { "payloadJson": "{\"predatorId\":101,\"targetId\":47}" }
→ 200  { "ok": true }
→ 502  { "ok": false, "error": "..." }
```

### 5.4 Plugin → SCRIMMAGE pub/sub (in-process)

When TopicTap receives a `PublishToTopic` RPC for `(GlobalNetwork, Commands/TargetAssignment)`:
1. **(gRPC thread)** Look up handler in `publish_registry`.
2. **(gRPC thread)** Handler calls `JsonStringToMessage(payload_json, &msg)`.
3. On parse error → `PublishToTopicResponse{ok: false, error: "json parse: ..."}`.
4. On success → enqueue `PendingPublish{network, topic, MessagePtr<TargetAssignment>}` onto `pending_publishes_` under mutex.
5. Return `PublishToTopicResponse{ok: true}` — "queued for publish" ack.
6. **(sim thread, next tick of `step_entity_interaction`)** Drain queue; for each pending publish, lazily `advertise(network, topic)` (cached), then `publisher->publish(typed_message)`. Bus delivers to all subscribers (RemotePredator).

`RemotePredator` then receives the message via its `subscribe<TargetAssignment>("GlobalNetwork", "Commands/TargetAssignment", ...)` callback, same in-process pub/sub flow as the existing Predator → CaptureEntity path.

## 6. Data flow

### 6.1 Cold start

No new cold-start steps. `PublishToTopic` is on the existing TopicTap gRPC server; available the moment the sim is running. `RemotePredator` is loaded by mission start.

### 6.2 Operator assigns a target

```
EntityInspectorCard          api               TopicTap (sim)        RemotePredator (sim)
  │                            │                    │                       │
operator clicks blue boid 47   │                    │                       │
selectedEntityId = 47          │                    │                       │
clicks "Set as predator        │                    │                       │
target"                        │                    │                       │
predatorId derived from        │                    │                       │
latest frame (team_id==2)      │                    │                       │
predatorId = 101               │                    │                       │
  ├─POST /api/commands/        │                    │                       │
  │  target-assignment─────────►                    │                       │
  │  {predatorId:101,          │                    │                       │
  │   targetId:47}             │                    │                       │
  │                            ├─PublishToTopic────►│                       │
  │                            │                    │ JsonStringToMessage   │
  │                            │                    │ → TargetAssignment    │
  │                            │                    │ → publish on          │
  │                            │                    │   GlobalNetwork       │
  │                            │                    │                       │ subscriber callback fires
  │                            │                    │                       │ msg.predator_id == own_id?
  │                            │                    │                       │ yes → assigned_target_id_=47
  │                            │  PublishToTopic    │                       │ next step_autonomy locks
  │                            │  Response{ok:true} │                       │ follow_id_ = 47, skips
  │                            │◄───────────────────┤                       │ auto-pick
  │  HTTP 200 {ok:true}        │                    │                       │ predator visibly turns
  │◄───────────────────────────┤                    │                       │ toward boid 47
  │                            │                    │                       │
button toggles to "Clear"      │                    │                       │
local optimistic state         │                    │                       │
```

### 6.3 Operator clears assignment

Same path with `targetId: 0`. Plugin publishes `TargetAssignment{predator_id: 101, target_id: 0}`. RemotePredator's callback sees `target_id == 0`, clears `assigned_target_id_`. Next `step_autonomy` resumes auto-picking.

### 6.4 Assigned target dies (predator catches it)

No new wire activity. RemotePredator checks `contacts_->count(assigned_target_id_) > 0` at the top of each tick; when the assigned target is captured, it disappears from `contacts_`, the plugin clears `assigned_target_id_`, and falls through to upstream auto-pick logic. Predator visibly switches to the nearest enemy.

Web: EntityInspectorCard enters `captured` mode for the assigned boid → after 2s clears selection → `assignedTargetId` reset on next frame derivation.

### 6.5 Predator dies

RemotePredator destroyed with the entity. Web's `predatorEntityId` derivation re-runs each frame and finds no team-2 entity → button disabled.

### 6.6 Mission restart with assignment in flight

Web local state still says "Clear" (stale) after sim restart. Operator never re-clicks → button label remains incorrect until the user selects a different entity. Acceptable v1 quirk; documented.

## 7. Error handling

| Failure | Handling |
|---|---|
| TopicTap gRPC unavailable (sim not running) | API returns 502 `{ok:false, error:"TopicTap unavailable: ..."}`. UI surfaces inline error under the button (3s auto-dismiss). |
| Sim running, but `(GlobalNetwork, Commands/TargetAssignment)` not in publish_registry | Plugin returns `{ok:false, error:"unknown publish topic '...'"}`. API forwards as 400. |
| Bad JSON payload | Plugin returns `{ok:false, error:"json parse: <detail>"}`. API forwards as 400. |
| `predatorId <= 0` | API rejects at typed endpoint (400 `{error:"predatorId must be > 0"}`) before calling gRPC. |
| `targetId < 0` | API rejects (400). Zero is valid (= clear). |
| Target id refers to non-existent entity | RemotePredator silently ignores in `step_autonomy`; falls through to auto-pick. No error surfaced. |
| Assignee target on same team as predator | Allowed by wire format and plugin. v1 doesn't gate this. |
| Multiple predators in mission | Each filters by predator_id; only addressed one accepts. UI v1 sends to "first predator found." |
| Two simultaneous assignments to same predator | Last one wins. No locking; assignment is single-int field. |

**Out of scope for v1:** auth/authz, rate limiting, command audit log, idempotency tokens.

## 8. Testing

In priority order:

**1. Manual smoke (must pass before v1 done):**
- `docker compose up --build` from `webapp/` clean.
- Browser → `http://localhost:5173` → start `predator_prey_boids.xml`.
- Wait for boids + 1 red predator visible. Predator auto-picks nearest blue.
- Click any blue boid → EntityInspectorCard shows it in `live` mode → "Set as predator target" button appears.
- Click button → button label flips to "Clear predator target". **Predator visibly turns toward the selected boid within ~1 second** and chases it.
- Pick a different blue boid → "Set as predator target" → predator switches.
- Click "Clear predator target" → predator visibly resumes auto-pick.
- Re-assign → wait for capture → captured boid disappears → predator visibly auto-picks next nearest.
- Topic feed (right rail) still shows `CaptureEntity` events from the kill (regression check on TopicTap v1).

**2. Two unit tests worth writing:**
- **C++ (overlay test/):** `test_target_assignment_dispatch` — feed JSON-encoded `TargetAssignment` through the publish_registry handler, verify it deserializes correctly. ~20 LoC.
- **C# (api):** unit-test the typed `/api/commands/target-assignment` endpoint constructs the right payload JSON and rejects `predatorId <= 0`. ~15 LoC. (Skip if no test project exists yet.)

**3. Skipped for hackathon:** integration tests of full gRPC + pub/sub round-trip, RemotePredator unit tests (verified by manual smoke), React component tests for the inspector card button.

## 9. Repository layout (additions and edits)

```
scrimmage/
└── webapp/
    ├── scrimmage-overlay/
    │   ├── msgs/
    │   │   ├── Commands.proto                                   ★ NEW
    │   │   └── CMakeLists.txt                                   ✏ edit (build new proto)
    │   ├── include/c2overlay/plugins/
    │   │   ├── interaction/TopicTap/
    │   │   │   ├── TopicTap.h                                   ✏ edit (publish_registry decl)
    │   │   │   └── TopicTapServiceImpl.h                        ✏ edit (PublishToTopic decl)
    │   │   └── autonomy/RemotePredator/                         ★ NEW dir
    │   │       ├── RemotePredator.h
    │   │       └── RemotePredator.xml
    │   ├── src/plugins/
    │   │   ├── interaction/TopicTap/
    │   │   │   ├── TopicTap.cpp                                 ✏ edit (publish_registry + handler)
    │   │   │   ├── TopicTapServiceImpl.cpp                      ✏ edit (PublishToTopic impl)
    │   │   │   └── CMakeLists.txt                               ✏ edit (link Commands proto)
    │   │   └── autonomy/RemotePredator/                         ★ NEW dir
    │   │       ├── CMakeLists.txt
    │   │       └── RemotePredator.cpp                           (copy-and-modify of upstream)
    │   ├── missions/
    │   │   └── predator_prey_boids.xml                          ✏ edit (Predator → RemotePredator)
    │   ├── test/
    │   │   ├── CMakeLists.txt                                   ✏ edit (add test_target_assignment_dispatch)
    │   │   └── test_target_assignment_dispatch.cpp              ★ NEW
    │   └── CMakeLists.txt                                       ✏ edit (add autonomy/RemotePredator subdir)
    ├── api/
    │   ├── Protos/scrimmage/
    │   │   ├── TopicTap.proto                                   ✏ edit (+ PublishToTopic RPC)
    │   │   └── Commands.proto                                   ★ NEW (mirror)
    │   ├── Endpoints.cs                                         ✏ edit (+ 2 POST endpoints)
    │   ├── TopicTapClient.cs                                    ✏ edit (+ PublishToTopicAsync helper)
    │   └── Api.csproj                                           ✏ edit (+ Commands.proto)
    └── web/src/
        ├── components/
        │   └── EntityInspectorCard.tsx                          ✏ edit (+ button + state)
        └── lib/
            └── commandsApi.ts                                   ★ NEW (POST helpers)
```

Approximate diff size: ~400 LOC C++ (mostly the Predator copy), ~80 LOC C#, ~80 LOC TS.

## 10. Future work / known limitations

### v2 (next iteration)

- **Generic "publish to a topic" UI** (the deferred B option from brainstorming) — topic dropdown (already populated from `OnTopicList`) + JSON editor + Send button. Reuses existing `POST /api/topics/.../publish`. Operator can manually exercise any registered publish topic.
- **Multi-predator UI** — when more than one team-2 entity exists, prompt operator to pick which predator (or assign to all). Web only; wire format already accommodates.
- **Command audit log** to Postgres: `command_event(mission_run_id, predator_id, target_id, issued_at, operator_id)`. Pairs with the topic-event audit log mentioned in TopicTap §10.
- **Confirmation modal** for assignments (currently fire-on-click).
- **Toast notifications** for assignment success/failure (currently inline error under button only).

### v3

- **Other commands** (`swap_team`, `set_speed`, `recall_to_base`, etc.) — each one is: new proto in Commands.proto + new entry in publish_registry + new typed REST endpoint + new UI affordance. The hardest part each time is the autonomy-side subscriber.
- **Generalized command pattern** — auto-discovery of available commands per autonomy plugin (so the UI can show "what can this entity do?"). Big lift, requires SCRIMMAGE core changes.

### Known v1 limitations

- Adding a new publishable topic requires editing `TopicTap.cpp` (publish_registry) and rebuilding the SCRIMMAGE container. Same recompile cost as observe-side taps.
- `RemotePredator` is a copy of upstream `Predator.cpp`; future SCRIMMAGE updates to Predator behavior require manual merge.
- UI button only appears when selected entity is on team 1 (the prey team). Hardcoded; not data-driven from autonomy capability metadata.
- Web's `predatorEntityId` is "first team-2 entity in latest frame." Multiple predators silently ignored beyond the first.
- Local UI assignment state isn't synced across browser refresh or multiple browser tabs.
- No way to query the current assignment from the sim. Web only knows what it last sent.

## 11. Open questions

None at design time. Likely implementation-time discoveries:
- Exact behavior of SCRIMMAGE's `subscribe<T>` callback when the message type doesn't match a published type on the same topic (probably dropped silently; verify).
- C++ refactoring shape for the `step_autonomy` early-out — the design uses `goto` for simplicity in pseudocode; production code likely extracts a `chase()` helper.
- Whether the existing `JsonStringToMessage` in protobuf C++ accepts both `predator_id` (snake_case) and `predatorId` (camelCase) — design assumes camelCase to match what the API sends; verify on first build.
