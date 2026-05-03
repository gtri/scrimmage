# SCRIMMAGE C2 UI

A web-based command-and-control operator console for [SCRIMMAGE](https://github.com/gtri/scrimmage) multi-agent simulations. Live 3D visualization on a real-Earth Cesium globe, with mission lifecycle controls, simulation-speed and pause control, post-mission reports, and bidirectional entity selection between the sidebar and the viewport.

Built as a hackathon prototype on top of a forked SCRIMMAGE base image.

---

## What it does

- **Live 3D visualization** of any SCRIMMAGE mission with a geographic origin. Entities render as colored points by team over real terrain (Cesium Ion world terrain).
- **Mission selector** — dropdown listing every `.xml` in `missions/`. Default is `predator_prey_boids.xml` at 10× speed.
- **Lifecycle controls** — Start / Pause / Resume / Stop. Pause uses `SIGSTOP`/`SIGCONT` to freeze the SCRIMMAGE process — sim state preserved exactly.
- **Operator-controlled simulation speed** (`time_warp`) via dropdown preset (0.5× to 50×). Applied at Start.
- **Bidirectional entity selection** — click a drone in the viewport to highlight its row in the sidebar (and vice versa). Cesium info-box shows entity details.
- **Auto-fit and recenter** — initial camera framing is oblique 3D over the mission's geographic origin. `⊕ Recenter` button refits to current entities on demand. After that, the operator owns the camera (no automatic motion).
- **Geocoded location badge** — origin shown as `📍 Camp Roberts · 35.7210, -120.7679 · 300m` via OpenStreetMap Nominatim reverse geocoding.
- **Auto-completion detection** — when SCRIMMAGE reaches its `end_condition`, the UI detects it via status polling and auto-pops the post-mission report.
- **Mission report modal** — captures SCRIMMAGE's stdout shutdown report (Metrics summaries) and surfaces it as a modal. Re-openable from the header.
- **Help overlay** — collapsible `?` panel in the viewer showing mouse + trackpad camera controls.

---

## Architecture

Four containers in one `docker-compose.yml`:

```
┌─ docker-compose.yml ─────────────────────────────────────────────┐
│                                                                   │
│   scrimmage          api               web              postgres │
│   ────────           ────              ────             ──────── │
│   GTRI base       .NET 10           React +           Postgres   │
│   image +         minimal API       Vite +            16-alpine  │
│   Flask           hosts gRPC        TypeScript +      (scaffolded│
│   launcher        server +          CesiumJS          unused)    │
│   sidecar         SignalR hub                                    │
│                                                                   │
│   :5050 HTTP      :8080 HTTP/1      :5173 Vite        :5432      │
│   :50051 gRPC     :50051 gRPC/h2c                                │
│                                                                   │
│   scrimmage  ─────► api  ◄────────  web                          │
│   pushes Frame    fans out via      subscribes to                │
│   protos via      SignalR           /hubs/frames                 │
│   gRPC h2c                          renders to Cesium            │
└──────────────────────────────────────────────────────────────────┘
```

**Cross-container protocols:**
- `scrimmage:50051 ◄── api:50051` — SCRIMMAGE pushes `Frame` protos to the API's hosted `ScrimmageService` gRPC server. HTTP/2 cleartext, separate from the REST/SignalR port.
- `scrimmage:5050 ◄── api` — REST: mission lifecycle (list/start/stop/pause/resume/status/report).
- `web ◄──► api` — REST + SignalR over HTTP/1.1 on `:8080`.

**Critical: the API hosts the gRPC server, scrimmage is the client.** This mirrors how SCRIMMAGE's native VTK viewer works.

---

## Tech stack

| Layer | Choice |
|---|---|
| Backend API | .NET 10 minimal API |
| Frontend | React + TypeScript + Vite |
| 3D Viewer | CesiumJS + Cesium Ion world terrain |
| Database | PostgreSQL 16 (scaffolded, unused in MVP) |
| Container orchestration | docker compose (single file) |
| SCRIMMAGE base image | `ghcr.io/gtri/scrimmage-24.04:latest` |
| Mission lifecycle sidecar | Python 3 + Flask |
| SCRIMMAGE ↔ API transport | gRPC (HTTP/2 cleartext) + HTTP (lifecycle) |
| API ↔ web transport | SignalR (frames) + REST (control) |
| Reverse geocoding | OpenStreetMap Nominatim (free, no API key) |

---

## Quick start

### Prerequisites
- Docker Desktop (or Docker Engine on Linux) with WSL2 backend on Windows
- Free [Cesium Ion access token](https://cesium.com/ion/) for terrain

### Run

```bash
# 1) Set your Cesium Ion token
cp webapp/.env.example webapp/.env
# edit webapp/.env: CESIUM_ION_TOKEN=eyJhbGc...

# 2) Bring up the stack (~5-10 min first time for the SCRIMMAGE base-image pull)
cd webapp
docker compose up --build

# 3) Open the operator console
# http://localhost:5173
```

Pick a mission from the dropdown (default: `predator_prey_boids.xml`), choose a speed, click `▶ Start`. Drones appear over Camp Roberts terrain.

### Smoke-test missions known to work

| Mission | Notes |
|---|---|
| `predator_prey_boids.xml` | Default. 100 prey + 1 predator. Boids autonomy. ~15s real time at 10×. |
| `capture-the-flag.xml` | Two teams of UAVs. Defaults to 20× speed. ~50s real time. |
| `auction_assign.xml` / `auction_assign2.xml` | Auction-based task assignment demos. |

Many other `.xml` missions in the SCRIMMAGE source require ROS, joystick devices, or specific build configs — they error gracefully with a launcher-side or scrimmage-side message.

### Operator controls in the viewer

- **One-finger drag** (trackpad) or **left-drag** (mouse) — pan
- **Pinch** or **wheel** — zoom
- **Ctrl + drag** or **right-drag** — tilt / rotate
- **Click a drone** — selects it (also highlights in sidebar, opens info box)
- **Click the `?` button** — toggle the controls cheat sheet

---

## Repository layout

```
webapp/
├── docker-compose.yml
├── README.md                     ← this file
├── .env.example                  ← CESIUM_ION_TOKEN placeholder
├── docs/
│   ├── specs/2026-05-02-scrimmage-c2-ui-design.md
│   └── plans/2026-05-02-scrimmage-c2-ui-mvp.md
├── scrimmage-runner/             ← scrimmage container build
│   ├── Dockerfile                  (FROM gtri/scrimmage-24.04 + python + flask)
│   ├── entrypoint.sh
│   └── launcher/
│       ├── app.py                  (Flask launcher: lifecycle + XML templating)
│       └── requirements.txt
├── api/                          ← .NET 10 minimal API
│   ├── Api.csproj
│   ├── Program.cs                  (Kestrel HTTP/1 + HTTP/2 split, CORS, services)
│   ├── FrameStreamService.cs       (hosts ScrimmageService gRPC server)
│   ├── FrameHub.cs                 (SignalR hub at /hubs/frames)
│   ├── Endpoints.cs                (REST: missions list/start/stop/pause/resume/status/report)
│   ├── AppDbContext.cs             (empty EF Core context — Postgres scaffold for v2)
│   ├── Dtos.cs
│   ├── Dockerfile
│   └── Protos/scrimmage/proto/     (.proto files mirrored from SCRIMMAGE source)
└── web/                          ← React + Vite frontend
    ├── package.json
    ├── package-lock.json
    ├── vite.config.ts              (vite-plugin-cesium for asset bundling)
    ├── index.html                  (CSS variables, Rajdhani + IBM Plex Mono fonts)
    ├── Dockerfile
    └── src/
        ├── AppShell.tsx            (header + sidebars + viewer + modal wiring)
        ├── components/
        │   ├── CesiumViewer.tsx    (Cesium scene, entity rendering, camera control)
        │   ├── MissionPicker.tsx   (mission/speed dropdowns + lifecycle buttons)
        │   ├── EntityList.tsx      (clickable sidebar list)
        │   ├── HelpOverlay.tsx     (camera-controls cheat sheet)
        │   ├── ReportModal.tsx     (post-mission report viewer)
        │   └── panels/Placeholder.tsx
        ├── hooks/useFrameStream.ts (SignalR client, throttled UI state)
        ├── lib/
        │   ├── api.ts              (REST client)
        │   ├── geocode.ts          (OSM Nominatim reverse geocoding)
        │   └── enuToCartesian.ts   (+ unit test — only nontrivial math)
        └── types.ts
```

---

## Known limitations and future work

The MVP is intentionally narrow. Several features were descoped or deferred:

**Operator-to-entity commands** — the original target was two commands (`target_assignment`, `swap_team`). Both require new C++ work in SCRIMMAGE (a custom autonomy plugin to listen on command topics, plus possibly core changes for mid-sim team swap). Out of scope for the 24-hour solo build; the right-rail panel is reserved for them.

**Live pub/sub topic stream** — operator subscribes to a drone's pub/sub topics. SCRIMMAGE's pub/sub is in-process only; this needs a new `TopicTap` interaction plugin in C++ to expose topics externally.

**Entity tagging and audit logging** — operator-side annotations and command history. Postgres is scaffolded but unused in MVP; these features land here.

**Frame scrubbing and replay** — SCRIMMAGE writes a binary log; could be tailed for replay. Useful for after-action review.

**Static mission geometry** — boundaries and flags (e.g., capture-the-flag) are SCRIMMAGE `entity_interaction` plugins, not entities, so they don't appear in the frame stream. Could be parsed from mission XML and sent as a one-shot scene-setup message, or pulled via `SendShapes`.

**Mission compatibility** — many mission XMLs require ROS, joystick devices, or specific build configurations and won't run in the demoable container. The dropdown shows all of them; non-demoable ones surface an error from the launcher or scrimmage. Future work could filter the dropdown to "demoable" missions only.

**Cleanup TODO**: the spec and plan in `docs/` were written before implementation surfaced several issues (proto namespace was `ScrimmageProto` not `Scrimmage_Proto`; `SendFrame` is unary; HTTP/2 needs a separate cleartext port; SCRIMMAGE_PLUGIN_PATH must include both `lib/` and `etc/`; missions without `<stream_ip>`/`<stream_port>` need them injected; `output_type` defaults to `summary` in some missions and must be forced to `all`). The implementation works; the docs lag.

See `docs/specs/2026-05-02-scrimmage-c2-ui-design.md` §10 for the full list.

---

## License

Inherits from the parent SCRIMMAGE fork.
