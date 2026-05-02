# SCRIMMAGE Pub/Sub Bridge — Design (TopicTap v1, observe-only)

**Date:** 2026-05-02
**Author:** Scott McCutchen
**Status:** Approved, ready for implementation planning
**Context:** Hackathon scope expansion on top of the SCRIMMAGE C2 UI MVP (`2026-05-02-scrimmage-c2-ui-design.md`). Solo developer.

## 1. Overview

A bridge that exposes SCRIMMAGE's in-process pub/sub topics to the operator console. v1 is **observe-only**: the operator picks a configured topic from a dropdown and watches messages stream in real time. Inject (operator publishes onto a topic) is explicitly deferred to v2 — the design leaves room for it without building it now.

The MVP design (§10) called this feature out as the deferred "TopicTap interaction plugin." This doc is that spec.

### Demo target

The MVP swaps from `capture-the-flag.xml` to `predator_prey_boids.xml` for this iteration. That mission has 100 blue boids being hunted by 1 red predator running the existing `Predator` autonomy plugin (`src/plugins/autonomy/Predator/Predator.cpp:60,113`), which already publishes `scrimmage_msgs::CaptureEntity` messages on `GlobalNetwork/CaptureEntity` whenever it gets within capture range. That gives us a real, semantically meaningful kill feed without writing any new sim-side autonomy code beyond the bridge plugin itself.

### In scope (v1)

- `TopicTap` SCRIMMAGE C++ plugin that subscribes to a configured allowlist of `(network, topic, type)` triples and exposes them via gRPC
- API-side gRPC client + SignalR hub + REST endpoint to list and stream topic messages
- Web-side right-rail panel (replacing the MVP's stub topics panel) with a topic dropdown and a scrolling event feed
- One configured tap for the demo: `(GlobalNetwork, CaptureEntity, scrimmage_msgs.CaptureEntity)`
- Mission XML edit to load `TopicTap` into `predator_prey_boids.xml`

### Out of scope (deferred)

- **Inject** (operator publishes onto a topic) — v2; design accommodates without building
- **Persistence to Postgres** (audit log) — v2; Postgres stays scaffolded-but-unused as in MVP
- **Auto-discovery of topics** — v3; requires SCRIMMAGE core change to expose pub/sub registry
- **Arbitrary protobuf types via reflection** — v3; v1 requires a recompile to add a new tappable type
- **Throttling/coalescing** — not needed at predator-capture rates; revisit if a high-rate topic is added later
- **Multi-topic simultaneous view** — v1 shows one topic at a time
- **Feed search/filter, replay, persistence across browser refresh**
- **Authn/authz on new RPCs and hub** — inherited gap from MVP

## 2. Locked-in design decisions

| # | Decision | Rationale |
|---|---|---|
| 1 | Direction: observe + inject overall, but **observe-only ships first** | Hackathon time budget |
| 2 | **Configured allowlist** in plugin XML, not auto-discovery | Auto-discovery requires SCRIMMAGE core change; out of budget |
| 3 | Bridge is a **new standalone `EntityInteraction` plugin** (`TopicTap`), not an extension of the existing `GRPCCommandString` | Keeps `GRPCCommandString`'s narrow contract intact; future inject features cohere in `TopicTap` |
| 4 | **Demo mission: `predator_prey_boids.xml`**, demo topic `GlobalNetwork/CaptureEntity` | Real, semantically meaningful pub/sub traffic from existing `Predator` autonomy |
| 5 | UX: **topic dropdown + scrolling feed** in the right-rail panel | Forward-compatible with v2 inject UI without throwaway work |
| 6 | **Fire-and-forget**, no Postgres persistence | Hackathon scope; audit log explicitly descoped in MVP |
| 7 | Wire payload as **`payload_json` string**, not `bytes` | Plugin knows the C++ type at compile time per tap; serialize at the source via `MessageToJsonString`; no protobuf reflection needed downstream |

## 3. Architecture (delta vs MVP)

```
┌─ docker-compose.yml ──────────────────────────────────────────────┐
│   ┌──────────────┐    ┌────────────┐    ┌──────────────┐         │
│   │  scrimmage   │    │    api     │    │     web      │         │
│   │              │    │            │    │              │         │
│   │  scrimmage   │◄───┤ gRPC frame │    │ Cesium       │         │
│   │   :50051     │    │ client     │    │ viewer       │         │
│   │              │    │            │    │              │         │
│   │  TopicTap    │◄───┤ gRPC topic │    │ TopicFeed    │         │
│   │   :60001 ★   │    │ client ★   │◄───┤ panel ★      │         │
│   │              │    │            │    │              │         │
│   │  launcher    │◄───┤ HTTP       │    │ MissionPicker│         │
│   │   :5050      │    │ FrameHub   │    │              │         │
│   └──────────────┘    │ TopicHub ★ │◄───┤ SignalR ×2 ★ │         │
│                       │ + REST     │    └──────────────┘         │
│                       └────────────┘                              │
└────────────────────────────────────────────────────────────────────┘
       ★ = new in this design
```

Three new things:
- A new SCRIMMAGE plugin (`TopicTap`) running its own gRPC server on `:60001`, separate from the existing frame stream on `:50051`
- A paired `BackgroundService` + SignalR `Hub` + REST endpoint in the .NET API
- A new right-rail panel and SignalR hook in the web app

The existing frame stream pipeline is untouched.

## 4. Components

### 4.1 `TopicTap` plugin (SCRIMMAGE C++, new)

**Location:** `src/plugins/interaction/TopicTap/` (mirrors `GRPCCommandString` layout).

**Plugin type:** `EntityInteraction`. Side thread runs a gRPC server (same model as `GRPCCommandString::run_server`).

**Plugin XML:**
```xml
<TopicTap>
  <ip>0.0.0.0</ip>
  <port>60001</port>
  <tap network="GlobalNetwork" topic="CaptureEntity" type="scrimmage_msgs.CaptureEntity"/>
</TopicTap>
```

**Init flow:**
1. Parse `ip`, `port`, and child `<tap>` elements.
2. For each tap, look up `type` in the static `type_name → add_tap<T>()` registry. Call the matching templated `add_tap<T>(network, topic)`, which:
   - Calls `subscribe<T>(network, topic, callback)` to register a SCRIMMAGE subscriber
   - The callback serializes the message payload via `google::protobuf::util::MessageToJsonString` and pushes a `TopicMessage{network, topic, type_name, t_sim, payload_json}` onto a thread-safe ring buffer (size 1000, drop-oldest).
3. Spawn the gRPC server thread.

**Static type registry:**
```cpp
// In TopicTap.cpp
static const std::map<std::string, AddTapFn> kTypeRegistry = {
    {"scrimmage_msgs.CaptureEntity", &TopicTap::add_tap<scrimmage_msgs::CaptureEntity>},
    // Add more types here as v2/v3 work lights them up.
};
```

Adding a new type = one line here + recompile. Documented limitation, acceptable for v1.

**gRPC service (`TopicTapService`):**
- `ListTopics(ListTopicsRequest) → TopicList` — returns the configured taps.
- `StreamTopic(StreamTopicRequest) → stream TopicMessage` — long-lived server stream, drains the ring buffer for messages matching `(request.network, request.topic)`.

**`step_entity_interaction`** is a no-op for v1 (the subscriber callback handles enqueue work directly). v2 inject will use `step` to drain a pending-publish queue and call `advertise().publish()`.

### 4.2 `TopicTapClient` (.NET `BackgroundService`, new)

**Location:** `webapp/api/TopicTapClient.cs`.

**Behavior:**
1. On startup (and after any disconnect), attempt to connect to `scrimmage:60001`.
2. On connect: call `ListTopics`, cache the result, push `OnTopicList(topics)` to `TopicHub`.
3. For each cached topic, open a `StreamTopic(network, topic)` gRPC server stream. On each message: forward to `TopicHub` via `OnTopicMessage`.
4. On any `RpcException` / `IOException`: log, clear cached topic list (push empty `OnTopicList` to hub), wait 500ms, retry. Same retry pattern as `ScrimmageGrpcClient`.

Cached topic list is also exposed to `Endpoints.cs` for the `GET /api/topics` REST endpoint (read-only access via DI'd singleton state).

### 4.3 `TopicHub` (SignalR Hub, new)

**Location:** `webapp/api/TopicHub.cs`. Path: `/hubs/topics`.

**Methods (server → client):**
- `OnTopicList(TopicSpecDto[])` — pushed when the client connects (replay current cache) and whenever the cache changes (on reconnect or mission start/stop).
- `OnTopicMessage(TopicMessageDto)` — pushed for every message received from any active `StreamTopic`.

No client → server methods in v1.

### 4.4 REST endpoint (new)

```
GET /api/topics
→ 200 [{ "network": "GlobalNetwork", "topic": "CaptureEntity", "typeName": "scrimmage_msgs.CaptureEntity" }]
```

Returns whatever `TopicTapClient` has cached. Empty array if sim is down or mission has no `TopicTap` plugin loaded.

Used for first-paint dropdown population before SignalR connects.

### 4.5 `TopicFeedPanel` (React, new)

**Location:** `webapp/web/src/components/panels/TopicFeedPanel.tsx`. Replaces the MVP stub topics panel.

**Layout:**
- Top: topic dropdown populated from `OnTopicList` (falls back to `GET /api/topics` for first paint).
- Below: scrolling feed, max 100 lines, newest at top.
- Empty states:
  - No topics configured: "No topics configured for this mission."
  - Topic selected, no messages yet: "Waiting for messages on `<topic>`…"

**Message rendering:**
- For `scrimmage_msgs.CaptureEntity`: human-readable formatter — `t=12.34s — entity 5 captured entity 47`.
- For any other type (when v2/v3 adds them): pretty-printed JSON.

A small `formatters` map keyed by `typeName` keeps the formatter logic out of the panel body.

### 4.6 `useTopicStream` hook (new)

**Location:** `webapp/web/src/hooks/useTopicStream.ts`.

Mirrors `useFrameStream`. Connects to `/hubs/topics` via `@microsoft/signalr` with `withAutomaticReconnect()`. Pushes incoming messages into module-level state. Exposes:
- `topics: TopicSpec[]` (current list from `OnTopicList`)
- `messages(network, topic): TopicMessage[]` (filtered, capped at 100)

### 4.7 Mission XML edit

`missions/predator_prey_boids.xml` — add inside `<runscript>`:
```xml
<entity_interaction>
  TopicTap
  <tap network="GlobalNetwork" topic="CaptureEntity" type="scrimmage_msgs.CaptureEntity"/>
</entity_interaction>
```

(SCRIMMAGE allows plugin-specific child elements inside the plugin tag — same pattern as existing plugins.)

## 5. Wire formats

### 5.1 SCRIMMAGE → API (new `.proto`)

```proto
// scrimmage/msgs/TopicTap.proto
syntax = "proto3";
package scrimmage_msgs;

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

Mirrored at `webapp/api/Protos/scrimmage/TopicTap.proto`. `Grpc.Tools` generates C# clients at build time, same pattern as the existing protos.

### 5.2 API → web (SignalR)

```ts
// On initial connect, mission start, mission stop
OnTopicList(topics: { network: string; topic: string; typeName: string }[])

// Per message
OnTopicMessage({
  network: string;
  topic: string;
  typeName: string;
  tSim: number;
  payloadJson: string;
})
```

Casing follows existing `FrameDto` convention (camelCase via .NET default `JsonOptions`).

### 5.3 REST

```
GET /api/topics
→ 200 [{ "network": "GlobalNetwork", "topic": "CaptureEntity", "typeName": "scrimmage_msgs.CaptureEntity" }]
```

### 5.4 Example payload

`payloadJson` for one capture event:
```json
{"sourceId":5,"targetId":47}
```

Rendered in the feed:
```
t=12.34s — entity 5 captured entity 47
```

## 6. Data flow

### 6.1 Cold start (delta vs MVP)

MVP cold-start unchanged through step 4. Add at step 5:

5. Browser loads → `TopicFeedPanel` fetches `GET /api/topics`. With no mission running, returns `[]`. Panel shows "No topics configured for this mission."

### 6.2 Mission start (predator_prey_boids)

```
Browser              api               launcher          scrimmage process
   │                  │                   │                   │
   ├─POST /start─────►│                   │                   │
   │                  ├─POST /start──────►│                   │
   │                  │                   ├─exec scrimmage────►(spawning)
   │                  │  {pid, origin}    │                   │
   │                  │◄──────────────────┤                   │
   │  {origin}        │  signal both grpc │                   │
   │◄─────────────────┤  clients to       │                   │
   │  configure       │  (re)connect      │                   │
   │  Cesium origin   ├─gRPC :50051──────────────────────────►(frame stream)
   │  (via existing)  ├─gRPC :60001──────────────────────────►(TopicTap)
   │                  │  ListTopics                            │
   │                  │  StreamTopic("GlobalNetwork",          │
   │                  │              "CaptureEntity")          │
   │  OnTopicList     │◄──────────────────────────────────────┤
   │◄─SignalR─────────┤                                       │
   │  populate        │                                       │
   │  dropdown        │                                       │
```

`MissionPicker` already triggers a frame-stream reconnect via the existing API signal; the same signal triggers `TopicTapClient` to (re)connect. Web does not need to refetch `GET /api/topics` — it gets the fresh list pushed via `OnTopicList`.

### 6.3 Live capture event

```
Predator.cpp                 TopicTap                   api                    web
                                │                        │                      │
publish(CaptureEntity{5,47}) ──►│                        │                      │
                                │ subscriber callback    │                      │
                                │ MessageToJsonString    │                      │
                                │ enqueue into ring buf  │                      │
                                │                        │                      │
                                │ gRPC stream emit       │                      │
                                │ ───────────────────────►                      │
                                │   TopicMessage{...}    │ TopicTapClient       │
                                │                        │ forward to hub       │
                                │                        │ ─────────────────────►
                                │                        │   OnTopicMessage     │
                                │                        │                      │ append to feed
```

End-to-end latency target: <100ms. Predator captures are visible-to-human events, not control loops.

### 6.4 Mission stop / restart

Same model as the existing frame stream: `TopicTapClient` enters its retry loop on disconnect; on the next mission start it reconnects, calls `ListTopics`, opens `StreamTopic`, pushes a fresh `OnTopicList` to the hub. Web treats a new `OnTopicList` as a session boundary and clears the feed.

## 7. Error handling

| Failure | Handling |
|---|---|
| `TopicTap` gRPC disconnect (sim down, mission restart) | `TopicTapClient` retry loop: any exception → log + 500ms wait + retry. Mirror of `ScrimmageGrpcClient`. |
| Mission has no `TopicTap` plugin loaded | `ListTopics` succeeds returning `[]`. Web shows "No topics configured for this mission." Not an error. |
| `<tap>` references a `type` not in the static registry | TopicTap init logs `[ERROR] TopicTap: unknown type 'foo.Bar', skipping`. Other taps still register. Mission still runs. |
| Subscriber callback overflow (publish rate > stream consume rate) | Bounded ring buffer (1000 messages). On full, drop oldest with one log line per N drops. Acceptable — observation is best-effort, not audit. |
| Web SignalR disconnect | `withAutomaticReconnect()`. On reconnect, hub replays current `OnTopicList`. Existing feed preserved on the client. |
| `MessageToJsonString` failure | TopicTap logs and skips the single message. Should never happen for well-formed protos. |
| Operator selects a topic with no messages yet | Empty feed with "Waiting for messages on `<topic>`…" placeholder. |

**Out of scope for v1 error handling:** authn/authz, rate limiting, retry budgets / circuit breakers on gRPC reconnect, message replay on disconnect, persistence.

## 8. Testing

In priority order, hackathon-sized:

**1. Manual smoke (the only one that absolutely must pass before v1 is "done"):**
- `docker compose up --build` clean from `webapp/`
- Browser → `http://localhost:5173`
- Mission dropdown shows `predator_prey_boids.xml`
- Click Start → within ~3s, blue boids visible in Cesium being chased by a red predator
- `TopicFeedPanel` dropdown populates with one entry: `GlobalNetwork / CaptureEntity`
- Within ~10s, capture events start appearing in the feed: `t=12.34s — entity 101 captured entity 47`
- Capture events line up with boids visibly disappearing from the Cesium view
- Click Stop → feed stays (frozen). Pick CTF mission → Start → feed clears, dropdown shows `[]`, panel says "No topics configured."
- Refresh browser mid-mission → reconnects, new captures resume appending

**2. One unit test:** TopicTap's `MessageToJsonString` round-trip on a `CaptureEntity` instance — confirms the JSON the operator sees matches what the proto carries. ~10 minutes. Sole purpose: catch a silent encoder-misconfig regression.

**3. Skipped for v1:** API integration tests, React component tests, end-to-end browser automation, plugin XML parsing tests.

## 9. Repository layout (additions only)

```
scrimmage/
├── include/scrimmage/plugins/interaction/TopicTap/
│   ├── TopicTap.h
│   └── TopicTapServiceImpl.h
├── src/plugins/interaction/TopicTap/
│   ├── CMakeLists.txt
│   ├── TopicTap.cpp
│   ├── TopicTap.xml
│   └── TopicTapServiceImpl.cpp
├── msgs/
│   └── TopicTap.proto
└── missions/
    └── predator_prey_boids.xml              (edit: add <entity_interaction>TopicTap</entity_interaction>)

webapp/
├── api/
│   ├── Protos/scrimmage/TopicTap.proto      (mirror of sim-side proto)
│   ├── TopicTapClient.cs                    (BackgroundService)
│   ├── TopicHub.cs                          (SignalR hub)
│   ├── Endpoints.cs                         (edit: + GET /api/topics)
│   ├── Dtos.cs                              (edit: + TopicMessageDto, TopicSpecDto)
│   └── Program.cs                           (edit: register hub + service)
└── web/src/
    ├── components/panels/TopicFeedPanel.tsx
    ├── hooks/useTopicStream.ts
    └── types.ts                             (edit: + TopicMessage, TopicSpec)
```

## 10. Future work

### v2 (next iteration — pairs with the descoped `target_assignment` / `swap_team` work)

- **Inject** — add unary RPC `PublishToTopic(network, topic, payload_json) → Empty` to `TopicTapService`. Add `POST /api/topics/{network}/{topic}/publish` to the API. Add a "Publish" button + JSON editor next to the topic dropdown. The autonomy plugins that are supposed to react still need their own subscriber wiring on the SCRIMMAGE side — that's a separate, larger spec.
- **Audit log to Postgres** — write each observed message to `topic_event(mission_run_id, network, topic, type_name, t_sim, payload_json, received_at)` from the API. Lights up the empty `AppDbContext`. Backfill panel feed on browser refresh from `GET /api/topics/{name}/recent`.

### v3

- **Auto-discovery of topics** — requires SCRIMMAGE core change: expose pub/sub registry on the `Network` base class. Removes the recompile-per-type cost.
- **Arbitrary protobuf types via reflection** — pairs with auto-discovery; lets operators tap topics whose types weren't known at plugin build time.
- **Throttling/coalescing** — relevant if a high-rate topic gets added (e.g., per-frame state broadcasts). Cap the wire rate at ~30 Hz per topic on the API side.

### Known v1 limitations

- Adding a new tappable type requires editing `TopicTap.cpp`'s static type registry and rebuilding the SCRIMMAGE container. Documented; acceptable for hackathon.
- Feed is in-memory only — browser refresh loses everything before the refresh.
- No filtering/searching within the feed.
- No multi-topic simultaneous view (one topic visible at a time; switch via dropdown).
- `TopicTap` is loaded/unloaded by editing mission XML; can't be toggled at runtime.

## 11. Open questions

None at design time. Likely implementation-time discoveries:
- Exact SCRIMMAGE plugin XML grammar for child elements inside `<entity_interaction>` (verify against existing plugins like `GRPCCommandString` that take parameters).
- Exact `Grpc.Tools` setup for the new mirror `.proto` (likely identical to existing protos in `webapp/api/Protos/scrimmage/`).
- Whether `subscribe<T>` on a topic with no publisher is silent or warns (affects log noise on missions that load TopicTap with taps for topics that aren't actually published).
