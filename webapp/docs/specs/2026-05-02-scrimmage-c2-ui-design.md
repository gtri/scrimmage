# SCRIMMAGE C2 UI — Design (MVP)

**Date:** 2026-05-02
**Author:** Scott McCutchen
**Status:** Approved, ready for implementation planning
**Context:** 24-hour hackathon. Solo developer on this work.

## 1. Overview

A web-based command-and-control UI built on top of a forked SCRIMMAGE multi-agent robotics simulator. The MVP is intentionally narrow: scaffold the full operator-console application and make the **3D live-visualization pane fully functional** for any pre-existing SCRIMMAGE mission with a geographic origin (capture-the-flag is the demo target). Future iterations layer on commands, pub/sub topic streams, entity tagging, and audit logging.

### In scope (MVP)
- Live 3D visualization of a running SCRIMMAGE mission in a browser
- Mission selector dropdown listing pre-existing mission XMLs
- Mission start/stop lifecycle controls
- Operator console layout with placeholder panels for future features
- Single docker compose stack runnable on Docker Desktop / WSL2

### Out of scope (MVP)
- Operator commands to entities (target assignment, swap teams) — descoped, see §10
- Live pub/sub topic stream visualization — descoped, see §10
- Entity tagging — descoped, see §10
- Audit logging to Postgres (Postgres is scaffolded but unused) — descoped, see §10
- Authentication / multi-user state sync
- Pause/resume controls and frame scrubbing — descoped, see §10
- Static mission geometry rendering (boundaries, flags) — descoped, see §10
- Mission XMLs without geographic origins (incompatible with Cesium viewer)

## 2. Tech stack

| Layer | Choice | Notes |
|---|---|---|
| Backend API | .NET 10 minimal API | Upgradable to controllers post-hackathon if needed |
| Frontend | React + TypeScript + Vite | Vite dev server in compose for hot reload |
| 3D Viewer | CesiumJS | Requires Cesium Ion API key (operator has one) |
| Database | PostgreSQL 16 | Scaffolded with empty schema, unused in MVP |
| Container orchestration | docker compose (single file) | Demo target is Docker Desktop on Windows / WSL2 |
| SCRIMMAGE base image | `ghcr.io/gtri/scrimmage-24.04:latest` | Layered with custom launcher sidecar |
| Mission lifecycle sidecar | Python + Flask | Tiny — runs as PID 1 in scrimmage container |
| API ↔ scrimmage transport | gRPC (frame stream) + HTTP (mission lifecycle) | gRPC stubs generated from existing `.proto` files |
| API ↔ web transport | SignalR (frame fan-out) + REST (mission control) | `@microsoft/signalr` on the client |
| React state | Plain React state (`useState`) | Cesium manages its own scene graph; React tree doesn't churn on frames |

## 3. Architecture

```
                                     User's Browser
                                          │
                                          │ http://localhost:5173
                                          ▼
┌─ docker-compose.yml ─────────────────────────────────────────────┐
│                                                                   │
│   ┌──────────────┐    ┌────────────┐    ┌──────────────┐        │
│   │  scrimmage   │    │    api     │    │     web      │        │
│   │              │    │            │    │              │        │
│   │  scrimmage   │◄───┤ gRPC       │    │ Cesium       │        │
│   │   process    │    │ client     │    │ viewer       │        │
│   │   :50051     │    │ (.NET 10)  │    │ (React+Vite) │        │
│   │              │    │            │    │              │        │
│   │  launcher    │◄───┤ HTTP       │    │ SignalR      │        │
│   │  sidecar     │    │ caller     │◄───┤ client       │        │
│   │  (Flask)     │    │            │    │              │        │
│   │  :5050       │    │ SignalR    │    │ REST calls   │        │
│   └──────────────┘    │ Hub        │    │              │        │
│         │             │ + REST API │◄───┤              │        │
│         │             └────────────┘    └──────────────┘        │
│         │                                                         │
│         │             ┌────────────┐                              │
│         └─unused MVP──►│  postgres  │                              │
│                       │  (empty)   │                              │
│                       └────────────┘                              │
└──────────────────────────────────────────────────────────────────┘
```

**Containers:** four — `scrimmage`, `api`, `web`, `postgres`.

**Cross-container protocols:**
- gRPC (`api → scrimmage:50051`) — frame streaming via SCRIMMAGE's existing `ScrimmageService` proto.
- HTTP (`api → scrimmage:5050`) — mission lifecycle via our launcher.
- SignalR + REST (`web ↔ api`) — operator-facing.

## 4. Components

### 4.1 `scrimmage` container

**Image:** built FROM `ghcr.io/gtri/scrimmage-24.04:latest`. Adds a small Python + Flask launcher sidecar.

**Process model:** entrypoint script `exec`s the launcher. Launcher is PID 1 and supervises the SCRIMMAGE process as a child — start/stop/restart on demand. No supervisord required.

**Launcher endpoints (HTTP on `:5050`):**

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/missions` | List `*.xml` files in `/scrimmage/missions/`, return JSON array |
| `POST` | `/missions/start` | Body: `{name}`. Kill child if running. Read mission XML; rewrite `network_gui="false"` → `"true"` (and set `enable_gui="false"` via override); parse `latitude_origin/longitude_origin/altitude_origin`. Write to `/tmp/active.xml`. Exec scrimmage with templated mission. Return `{status, pid, mission, origin: {lat, lon, alt}}` |
| `POST` | `/missions/stop` | SIGTERM the child, return `{status}` |
| `GET` | `/status` | Return currently running mission name + uptime, or `idle` |

**Failure modes the launcher must handle:**
- Mission file not found → 404 with message.
- Mission XML missing one of `latitude_origin/longitude_origin/altitude_origin` → 400 with explicit "no geographic origin" message.
- Scrimmage process exits unexpectedly → reap child, return to idle state.

### 4.2 `api` container (.NET 10)

**Project shape:** a single ASP.NET Core minimal-API project (~5 source files).

**Components:**

- **`ScrimmageGrpcClient` (`BackgroundService`)** — Maintains a streaming gRPC connection to `scrimmage:50051`. On each `Frame` received, transforms `Contact[]` into the `FrameDto` (see §6) and pushes to the SignalR hub. Reconnect loop on disconnect: 500ms backoff, retry forever (mission restarts are normal disconnects).
- **`FrameHub : Hub`** — SignalR hub at `/hubs/frames`. Single broadcast method `OnFrame(FrameDto)`. Web clients subscribe.
- **Minimal API endpoints (no controllers):**
  - `GET /api/missions` → proxy to launcher's `GET /missions`
  - `POST /api/missions/start` → proxy to launcher's `POST /missions/start`, then signal `ScrimmageGrpcClient` to reset its connection
  - `POST /api/missions/stop` → proxy to launcher's `POST /missions/stop`
  - `GET /api/status` → proxy to launcher's `GET /status`
- **`AppDbContext` (EF Core + Npgsql)** — empty DbContext, no entities. Connection string from `ConnectionStrings__Default` env var. No migrations yet; first migration lands when commands/audit ship.
- **gRPC stubs** — `Scrimmage.proto`, `Frame.proto`, `Contact.proto`, `State.proto`, `ID.proto`, `Vector3d.proto`, `Quaternion.proto` referenced from the SCRIMMAGE source tree. `Grpc.Tools` MSBuild target generates C# clients at build time.

### 4.3 `web` container (React + Vite + CesiumJS)

**Project shape:** Vite + React + TypeScript template.

**Components:**

- **`AppShell`** — header (mission selector + start/stop + status badge), left sidebar (entity list, populated from latest frame), main pane = Cesium viewer, right rail = stub panels for future features (commands / topics / tags). Chrome exists day one so future features have a home.
- **`CesiumViewer`** — wraps `Cesium.Viewer`. Initialized with mission origin (lat/lon/alt) + Ion terrain. Maintains a `Map<entityId, Cesium.Entity>` updated from frames. Per-frame: convert ENU `(x,y,z)` → `Cartesian3` using mission origin, set `entity.position`. Orientation from quaternion → `Cesium.Quaternion`. Color by `teamId` (1 = blue, 2 = red, others = neutral palette). Remove entity when `active=false`.
- **`useFrameStream`** — React hook that connects to `/hubs/frames` via `@microsoft/signalr` (with `withAutomaticReconnect()`), pushes incoming frames into module-level state.
- **`MissionPicker`** — fetches `/api/missions` on mount, dropdown + Start/Stop buttons. Shows currently active mission name + status badge.
- **`enuToCartesian(originLatLonAlt, x, y, z)` util** — uses `Cesium.Transforms.eastNorthUpToFixedFrame` matrix. ~20 lines, isolated, the one piece of nontrivial math in the codebase. **Has a unit test** (see §8).

### 4.4 `postgres` container

**Image:** `postgres:16-alpine`. Init script creates database `c2`. **No tables for MVP.** API holds an EF Core `DbContext` referencing it but does not migrate anything yet. This is the future home of audit log, tagged entities, and command history.

## 5. Data flow

### 5.1 Cold start (`docker compose up`)

1. `postgres` starts.
2. `scrimmage` container starts: launcher Flask app binds `:5050`. **No scrimmage process running** — idle state.
3. `api` starts: gRPC client opens to `scrimmage:50051`, gets connection-refused, enters retry loop with 500ms backoff. SignalR hub and REST endpoints come up immediately.
4. `web` starts: Vite dev server on `:5173`.
5. User opens browser → React loads → `MissionPicker` fetches `GET /api/missions` → API proxies to launcher → dropdown populates.

### 5.2 Mission start lifecycle

```
Browser              api               launcher          scrimmage process
   │                  │                   │                   │
   ├─POST /start─────►│                   │                   │
   │                  ├─POST /start──────►│                   │
   │                  │                   ├─SIGTERM old──────►X
   │                  │                   ├─template XML      │
   │                  │                   ├─parse origin      │
   │                  │                   ├─exec scrimmage────►(spawning)
   │                  │  {pid, origin}    │                   │
   │                  │◄──────────────────┤                   │
   │  {origin}        │  signal grpc      │                   │
   │◄─────────────────┤  client to        │                   │
   │  configure       │  (re)connect      │                   │
   │  Cesium viewer   │                   │                   │
   │  to origin       ├─gRPC connect──────────────────────────►:50051
   │                  │◄─────Frame stream─────────────────────┤
   │◄─SignalR frames──┤                                       │
   │                  │                                       │
   ▼                  ▼                                       ▼
 update              fan out                              sim ticks
 Cesium                                                  every 0.1s
 entities
```

The launcher's response includes the **mission origin** (parsed from XML: `latitude_origin`, `longitude_origin`, `altitude_origin`). The API forwards it to the browser; React uses it to anchor the Cesium camera and to do ENU → Cartesian conversion for every entity position.

### 5.3 Frame cadence

Capture-the-flag has `dt="0.1"` (10 Hz sim) and `gui_update_period="10ms"` (~100 Hz emit). With ~20 entities, ~2000 contact-updates/sec across the wire. Trivial for loopback gRPC and SignalR — no throttling needed for MVP. If perf issues appear, easy fix is coalescing on the API side (publish at most every 33 ms = 30 Hz to clients). Tunable, not implemented MVP.

### 5.4 Mission stop / restart

- **Stop:** browser → API → launcher → SIGTERM scrimmage. gRPC stream ends; API client returns to retry loop. Web shows "idle" badge.
- **Restart:** same path as start. The API gRPC client treats it as a normal reconnect — no special handling beyond the retry loop already in place.

### 5.5 Entity disappearance

A `Contact.active` flag goes false when an entity is destroyed (e.g., collision). Cesium client removes that entity from its map. Important for capture-the-flag where collisions kill drones.

## 6. Wire formats

### 6.1 SCRIMMAGE → API (gRPC, existing protos)

`Frame { double time; repeated Contact contact; }`
`Contact { ID id; State state; ContactType type; bool active; }`
`ID { int32 id; int32 sub_swarm_id; int32 team_id; }`
`State { Vector3d position; Quaternion orientation; Vector3d linear_velocity; Vector3d angular_velocity; }`

### 6.2 API → Web (SignalR, our DTO)

```json
{
  "time": 12.3,
  "entities": [
    {
      "id": 1,
      "teamId": 1,
      "subSwarmId": 0,
      "type": "AIRCRAFT",
      "active": true,
      "position": { "x": 400.1, "y": 0.2, "z": 200.0 },
      "velocity": { "x": -10.5, "y": 0.0, "z": 0.0 },
      "orientation": { "w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0 }
    }
  ]
}
```

Position/velocity in **ENU meters from mission origin**. Orientation as quaternion (w, x, y, z). `type` mapped from `ContactType` enum to a string for readability.

### 6.3 Mission start response (REST)

```json
{
  "status": "started",
  "pid": 42,
  "mission": "capture-the-flag.xml",
  "origin": { "lat": 35.721025, "lon": -120.767925, "alt": 300 }
}
```

## 7. Error handling

| Failure | Handling |
|---|---|
| API ↔ scrimmage gRPC disconnect | `ScrimmageGrpcClient` retry loop: open stream, on any exception (`RpcException`, `IOException`) log + 500ms wait + retry. No backoff escalation. |
| Launcher 4xx/5xx (mission not found, scrimmage crash, port conflict) | API forwards as HTTP error; web shows toast + sets status badge red. Operator can pick a different mission. |
| Mission XML missing geographic origin | Launcher returns `400 {error}`; web shows clear message. Acceptable — not all missions are demoable. |
| SignalR disconnect (web ↔ API) | `@microsoft/signalr` `withAutomaticReconnect()`. Built in. |
| Cesium Ion failure (bad/missing key, asset load fail) | `CesiumViewer` catches init error; shows banner: "3D map unavailable — check Ion API key." Doesn't crash the app. |
| Postgres down | API logs warning at startup but doesn't fail. We're not reading/writing DB in MVP. Will switch to fail-fast once we are. |

**Out of scope for MVP error handling:** auth/authz, rate limiting on launcher, retry budgets / circuit breakers on gRPC reconnect, frame validation.

## 8. Testing

In priority order:

**1. Manual smoke test** — the only one that absolutely must pass before calling MVP done.
- `docker compose up --build` from `webapp/` clean.
- Browser → `http://localhost:5173`.
- Mission dropdown populated; `capture-the-flag.xml` visible.
- Click Start → within ~3 seconds, blue + red drones visible in Cesium over Camp Roberts terrain.
- Drones move; collisions cause entities to disappear.
- Click Stop → drones freeze/disappear; pick a different mission → Start → new mission renders.
- Refresh browser mid-mission → reconnects, frames resume.

**2. One unit test worth writing** — `enuToCartesian(origin, x, y, z)` round-trip on whichever side does the conversion (web). Only nontrivial math in the codebase; a coordinate bug would silently put drones in the ocean. ~15 minutes.

**3. Skipped for hackathon, deferred post-MVP:** API integration tests, React component tests, launcher unit tests, end-to-end browser automation.

## 9. Repository layout

New code lives under a single top-level `webapp/` folder in the SCRIMMAGE fork — keeps it isolated from upstream-mergeable directories.

```
C:\Git\kinetas\scrimmage\
├── (existing SCRIMMAGE source untouched)
└── webapp\
    ├── docker-compose.yml
    ├── README.md
    ├── docs\
    │   └── specs\
    │       └── 2026-05-02-scrimmage-c2-ui-design.md   (this file)
    ├── scrimmage-runner\
    │   ├── Dockerfile                  (FROM gtri/scrimmage-24.04 + launcher)
    │   ├── launcher\
    │   │   ├── app.py                  (Flask launcher)
    │   │   └── requirements.txt
    │   └── entrypoint.sh
    ├── api\
    │   ├── Dockerfile                  (.NET 10 SDK + runtime)
    │   ├── Api.csproj
    │   ├── Program.cs                  (minimal API setup)
    │   ├── ScrimmageGrpcClient.cs      (BackgroundService)
    │   ├── FrameHub.cs                 (SignalR hub)
    │   ├── Endpoints.cs                (mission/status REST endpoints)
    │   ├── Dtos.cs                     (FrameDto, EntityDto, MissionStartResponse)
    │   ├── AppDbContext.cs             (empty EF Core context)
    │   └── Protos\                     (referenced .proto files; Grpc.Tools generates C#)
    └── web\
        ├── Dockerfile                  (Node + Vite dev server)
        ├── package.json
        ├── vite.config.ts
        ├── index.html
        ├── src\
        │   ├── main.tsx
        │   ├── AppShell.tsx
        │   ├── components\
        │   │   ├── CesiumViewer.tsx
        │   │   ├── MissionPicker.tsx
        │   │   ├── EntityList.tsx
        │   │   └── panels\             (placeholder panels for future features)
        │   ├── hooks\
        │   │   └── useFrameStream.ts
        │   ├── lib\
        │   │   ├── enuToCartesian.ts
        │   │   └── enuToCartesian.test.ts
        │   └── types.ts                (FrameDto, EntityDto mirrored from API)
        └── .env.example                (CESIUM_ION_TOKEN placeholder)
```

## 10. Future work / Known limitations / Descoped items

Captured here so the next iteration has a roadmap.

### Descoped commands (deferred — see also project memory)
- **`target_assignment`** — change a specific drone's adversarial target. Requires a custom autonomy plugin on the SCRIMMAGE side that subscribes to the command topic and mutates entity state.
- **`swap_team`** — flip a specific drone between friendly/enemy. May require SCRIMMAGE core changes since `team_id` is set at mission init and read by collision rules, contact filtering, and viewer coloring.

Both commands need C++ work in SCRIMMAGE itself, which doesn't fit a 24h solo build alongside the UI.

### Descoped UI features (deferred to post-MVP iterations)
- **Live pub/sub topic stream** — operator picks a topic for an entity and watches messages stream. Requires a new `TopicTap` interaction plugin in SCRIMMAGE C++ to expose in-process pub/sub topics externally.
- **Entity tagging** — operator-side annotations. Pure browser-side; needs Postgres-backed persistence.
- **Audit log to Postgres** — every command + tag + mission start/stop persisted with timestamp + operator.
- **Pause / resume controls** — SCRIMMAGE supports pause but it's keyboard-driven in the native viewer; we'd need launcher endpoints + signal handling. Useful enough that it's worth doing early in v2.
- **Frame scrubbing / replay** — SCRIMMAGE writes a binary frames log; we could read it for replay alongside the live stream. Useful for after-action review.

### Known MVP rendering gaps
- **Static mission geometry not rendered** (boundaries, flags). They're `entity_interaction` plugins, not `Contact`s, so they don't appear in the frame stream. To add later: either parse them from the mission XML on the API side and send a one-shot "scene setup" message at mission start, or wire up SCRIMMAGE's `SendShapes` gRPC method (already exists in `ScrimmageService`).
- **Visual models not used.** SCRIMMAGE entities have `<visual_model>zephyr-blue</visual_model>` etc. We render generic colored cones in MVP; future work could load the corresponding meshes in Cesium.

### Other future work
- Auth (multi-operator)
- Multi-client cursor/selection sync via SignalR groups
- Mission selector UX: thumbnails, metadata, "demoable" filter (hide missions without geographic origins)
- Throttling/coalescing the SignalR fan-out if frame rate becomes a problem
- Switch React state from `useState` to Zustand once multiple panels need to read frame state

## 11. Open questions

None at design time — all clarifications resolved during brainstorming. Implementation may surface new ones (likely candidates: SCRIMMAGE override-flag CLI syntax for `network_gui`, exact Vite + Cesium asset config, exact `Grpc.Tools` setup with cross-project `.proto` references).
