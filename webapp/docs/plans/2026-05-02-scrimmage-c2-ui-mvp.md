# SCRIMMAGE C2 UI MVP — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a four-container web C2 UI that visualizes a running SCRIMMAGE mission live in a browser, with a mission selector dropdown that can start/stop any pre-existing mission with a geographic origin.

**Architecture:** `docker compose` brings up four containers — `scrimmage` (GTRI base image + Flask launcher sidecar), `api` (.NET 10 minimal API: gRPC client of SCRIMMAGE's frame stream + SignalR fan-out + REST mission lifecycle), `web` (React + Vite + Cesium 3D viewer), `postgres` (scaffolded but unused in MVP). Frame data flows: scrimmage → gRPC :50051 → API → SignalR → React → Cesium entities, with mission origin parsed from XML and used for ENU→Cartesian conversion in-browser.

**Tech Stack:** .NET 10 minimal API, React + TypeScript + Vite, CesiumJS (with Ion terrain), PostgreSQL 16, Python 3 + Flask, Docker Compose, gRPC, SignalR.

**Working directory:** All `git` commands run from `C:\Git\kinetas\scrimmage\` (the SCRIMMAGE fork). All file paths in this plan are relative to that root unless absolute.

**Spec:** `webapp/docs/specs/2026-05-02-scrimmage-c2-ui-design.md`

---

## Pre-flight

Before starting, verify:

- [ ] Docker Desktop running (with WSL2 backend on Windows)
- [ ] You have a Cesium Ion API key (free at cesium.com/ion)
- [ ] The fork at `C:\Git\kinetas\scrimmage` is on your working branch with a clean tree (`git status`)
- [ ] You can pull `ghcr.io/gtri/scrimmage-24.04:latest` (`docker pull ghcr.io/gtri/scrimmage-24.04:latest`)

---

## Task 1: Foundation — webapp skeleton, .gitignore, docker-compose.yml

**Files:**
- Create: `webapp/.gitignore`
- Create: `webapp/README.md`
- Create: `webapp/.env.example`
- Create: `webapp/docker-compose.yml`

- [ ] **Step 1: Create `webapp/.gitignore`**

```
# Build outputs
**/bin/
**/obj/
**/dist/
**/node_modules/
**/__pycache__/

# Local env (never commit Cesium token)
.env
*.local

# Logs
*.log
```

- [ ] **Step 2: Create `webapp/README.md`** (stub — finalized in Task 20)

```markdown
# SCRIMMAGE C2 UI

Web-based command-and-control UI for SCRIMMAGE missions. See `docs/specs/` for design.

## Quick start
1. Copy `.env.example` to `.env` and add your Cesium Ion token.
2. `docker compose up --build`
3. Open http://localhost:5173
```

- [ ] **Step 3: Create `webapp/.env.example`**

```
# Cesium Ion access token (get one free at https://cesium.com/ion/)
CESIUM_ION_TOKEN=your_token_here
```

- [ ] **Step 4: Create `webapp/docker-compose.yml`** (full skeleton — services filled in later tasks add images and command details)

```yaml
services:

  scrimmage:
    build:
      context: ./scrimmage-runner
    container_name: c2-scrimmage
    ports:
      - "50051:50051"   # gRPC frame stream (debug)
      - "5050:5050"     # launcher HTTP (debug)
    networks: [c2net]

  api:
    build:
      context: ./api
    container_name: c2-api
    depends_on: [scrimmage, postgres]
    environment:
      - ASPNETCORE_URLS=http://+:8080
      - SCRIMMAGE_GRPC_ADDR=http://scrimmage:50051
      - SCRIMMAGE_LAUNCHER_URL=http://scrimmage:5050
      - ConnectionStrings__Default=Host=postgres;Database=c2;Username=c2;Password=c2pass
    ports:
      - "8080:8080"
    networks: [c2net]

  web:
    build:
      context: ./web
    container_name: c2-web
    depends_on: [api]
    environment:
      - VITE_API_URL=http://localhost:8080
      - VITE_CESIUM_ION_TOKEN=${CESIUM_ION_TOKEN}
    ports:
      - "5173:5173"
    networks: [c2net]

  postgres:
    image: postgres:16-alpine
    container_name: c2-postgres
    environment:
      - POSTGRES_DB=c2
      - POSTGRES_USER=c2
      - POSTGRES_PASSWORD=c2pass
    ports:
      - "5432:5432"
    networks: [c2net]
    volumes:
      - c2pgdata:/var/lib/postgresql/data

networks:
  c2net:

volumes:
  c2pgdata:
```

- [ ] **Step 5: Verify the compose file parses**

Run: `cd webapp && docker compose config`
Expected: prints the resolved compose config (the build contexts will warn that they don't exist yet — fine for now).

- [ ] **Step 6: Commit**

```bash
git add webapp/
git commit -m "feat(c2-ui): scaffold webapp directory with compose skeleton"
```

---

## Task 2: Scrimmage runner — Dockerfile + Flask launcher

**Files:**
- Create: `webapp/scrimmage-runner/Dockerfile`
- Create: `webapp/scrimmage-runner/entrypoint.sh`
- Create: `webapp/scrimmage-runner/launcher/app.py`
- Create: `webapp/scrimmage-runner/launcher/requirements.txt`

- [ ] **Step 1: Create `webapp/scrimmage-runner/Dockerfile`**

```dockerfile
FROM ghcr.io/gtri/scrimmage-24.04:latest

USER root

# Install Python + Flask for the launcher sidecar
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Set up venv to keep launcher deps isolated
RUN python3 -m venv /opt/launcher-venv
ENV PATH="/opt/launcher-venv/bin:${PATH}"

COPY launcher/requirements.txt /opt/launcher/requirements.txt
RUN pip install --no-cache-dir -r /opt/launcher/requirements.txt

COPY launcher/ /opt/launcher/
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

# Make sure scrimmage's env is sourced for child processes
ENV MISSIONS_DIR=/root/scrimmage/scrimmage/missions

EXPOSE 50051 5050

ENTRYPOINT ["/opt/entrypoint.sh"]
```

- [ ] **Step 2: Create `webapp/scrimmage-runner/entrypoint.sh`**

```bash
#!/usr/bin/env bash
set -euo pipefail

# Source scrimmage env so the `scrimmage` binary + plugin paths are available to the launcher
if [ -f /root/.scrimmage/setup.bash ]; then
  set +u
  source /root/.scrimmage/setup.bash
  set -u
fi

# Hand off to the Flask launcher (PID 1)
exec python -m launcher.app
```

> Note: the exact path to the scrimmage env-setup script depends on the base image's install layout. If `/root/.scrimmage/setup.bash` doesn't exist, search the container with `docker run --rm -it ghcr.io/gtri/scrimmage-24.04:latest find / -name "setup.bash" 2>/dev/null` and update the path. The scrimmage binary should already be on PATH in this image, but plugin paths matter.

- [ ] **Step 3: Create `webapp/scrimmage-runner/launcher/requirements.txt`**

```
flask==3.0.0
```

- [ ] **Step 4: Create `webapp/scrimmage-runner/launcher/__init__.py` (empty)**

```
```

- [ ] **Step 5: Create `webapp/scrimmage-runner/launcher/app.py`**

```python
"""Flask launcher sidecar for SCRIMMAGE.

Runs as PID 1, exposes HTTP on :5050 to start/stop scrimmage processes
and list available missions. Templates the chosen mission XML to enable
gRPC streaming on :50051 before launching.
"""
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from xml.etree import ElementTree as ET

from flask import Flask, jsonify, request

MISSIONS_DIR = Path(os.environ.get("MISSIONS_DIR", "/root/scrimmage/scrimmage/missions"))
ACTIVE_MISSION_PATH = Path("/tmp/active_mission.xml")

app = Flask(__name__)

_state = {"proc": None, "mission": None, "started_at": None, "origin": None}


def _running() -> bool:
    p = _state["proc"]
    return p is not None and p.poll() is None


def _stop():
    p = _state["proc"]
    if p and p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    _state.update({"proc": None, "mission": None, "started_at": None, "origin": None})


def _template_mission(src: Path) -> dict:
    """Copy mission XML to /tmp with network_gui flipped on. Return parsed origin."""
    text = src.read_text()
    # Flip network_gui="false" to "true" (regex tolerates whitespace around =)
    text = re.sub(r'network_gui\s*=\s*"false"', 'network_gui="true"', text)
    # Force enable_gui off so we don't try to open VTK in a headless container
    text = re.sub(r'enable_gui\s*=\s*"\$\{enable_gui=true\}"',
                  'enable_gui="false"', text)
    text = re.sub(r'enable_gui\s*=\s*"true"', 'enable_gui="false"', text)
    ACTIVE_MISSION_PATH.write_text(text)

    # Parse origin from the templated file
    root = ET.fromstring(text)
    def _find(tag):
        node = root.find(tag)
        return float(node.text) if node is not None and node.text else None
    return {
        "lat": _find("latitude_origin"),
        "lon": _find("longitude_origin"),
        "alt": _find("altitude_origin"),
    }


@app.get("/missions")
def list_missions():
    if not MISSIONS_DIR.exists():
        return jsonify({"error": f"missions dir not found: {MISSIONS_DIR}"}), 500
    files = sorted(p.name for p in MISSIONS_DIR.glob("*.xml"))
    return jsonify(files)


@app.post("/missions/start")
def start_mission():
    body = request.get_json(silent=True) or {}
    name = body.get("name")
    if not name:
        return jsonify({"error": "missing 'name'"}), 400
    src = MISSIONS_DIR / name
    if not src.exists():
        return jsonify({"error": f"mission not found: {name}"}), 404

    # Stop anything currently running
    _stop()

    # Template + parse origin
    try:
        origin = _template_mission(src)
    except Exception as e:
        return jsonify({"error": f"failed to template mission: {e}"}), 500
    if not all(origin.values()):
        return jsonify({"error": "mission has no geographic origin (lat/lon/alt) — incompatible with Cesium viewer"}), 400

    # Launch scrimmage
    proc = subprocess.Popen(
        ["scrimmage", str(ACTIVE_MISSION_PATH)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    _state.update({
        "proc": proc,
        "mission": name,
        "started_at": time.time(),
        "origin": origin,
    })

    return jsonify({
        "status": "started",
        "pid": proc.pid,
        "mission": name,
        "origin": origin,
    })


@app.post("/missions/stop")
def stop_mission():
    _stop()
    return jsonify({"status": "stopped"})


@app.get("/status")
def status():
    if not _running():
        return jsonify({"status": "idle"})
    return jsonify({
        "status": "running",
        "mission": _state["mission"],
        "uptime_s": time.time() - _state["started_at"],
        "origin": _state["origin"],
    })


def _shutdown(*_):
    _stop()
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    app.run(host="0.0.0.0", port=5050)
```

- [ ] **Step 6: Build the scrimmage runner image**

Run: `cd webapp && docker compose build scrimmage`
Expected: image builds successfully. If the base image pull is slow, that's normal (~1.5 GB).

- [ ] **Step 7: Commit**

```bash
git add webapp/scrimmage-runner/
git commit -m "feat(c2-ui): add scrimmage-runner Dockerfile + Flask launcher"
```

---

## Task 3: Smoke-test the scrimmage runner end-to-end

This is a checkpoint — we verify the launcher actually starts a mission and the gRPC port becomes reachable before building anything that depends on it.

**Files:** none (verification only)

- [ ] **Step 1: Bring up just the scrimmage container**

Run: `cd webapp && docker compose up scrimmage`
Expected: launcher logs "* Running on http://0.0.0.0:5050". Leave this running in one terminal.

- [ ] **Step 2: List missions (in a second terminal)**

Run: `curl http://localhost:5050/missions`
Expected: a JSON array including `"capture-the-flag.xml"` among others.

- [ ] **Step 3: Start capture-the-flag**

Run: `curl -X POST http://localhost:5050/missions/start -H "Content-Type: application/json" -d "{\"name\":\"capture-the-flag.xml\"}"`
Expected: JSON like `{"status":"started","pid":N,"mission":"capture-the-flag.xml","origin":{"lat":35.721025,"lon":-120.767925,"alt":300}}`. The scrimmage container logs should now show scrimmage startup output.

- [ ] **Step 4: Verify the gRPC port is open**

Run (Windows PowerShell): `Test-NetConnection -ComputerName localhost -Port 50051`
Or (bash): `nc -zv localhost 50051`
Expected: connection succeeds.

- [ ] **Step 5: Check status, then stop**

Run: `curl http://localhost:5050/status` → expect `running`
Run: `curl -X POST http://localhost:5050/missions/stop` → expect `{"status":"stopped"}`
Run: `curl http://localhost:5050/status` → expect `idle`

- [ ] **Step 6: Tear down**

Ctrl-C the compose terminal, then `docker compose down` to clean up.

- [ ] **Step 7: Commit (no code changes — just a checkpoint marker)**

If any tweaks were needed to the launcher in this task to get it working, commit them now:

```bash
git add -A
git commit -m "fix(c2-ui): smoke-test fixes for scrimmage-runner" --allow-empty
```

> **If this task fails:** the most likely culprits are (a) `setup.bash` path wrong in `entrypoint.sh` (run `docker run --rm -it c2-scrimmage bash` and `find / -name setup.bash`), (b) `MISSIONS_DIR` path wrong (search the container for `capture-the-flag.xml`), or (c) scrimmage binary not on PATH (try `docker run --rm -it c2-scrimmage which scrimmage`). Fix and re-run before moving on. **Do not start the API tasks until this works.**

---

## Task 4: API project scaffold + Dockerfile

**Files:**
- Create: `webapp/api/Api.csproj`
- Create: `webapp/api/Program.cs` (minimal stub for now)
- Create: `webapp/api/Dockerfile`
- Create: `webapp/api/.dockerignore`

- [ ] **Step 1: Create `webapp/api/Api.csproj`**

```xml
<Project Sdk="Microsoft.NET.Sdk.Web">

  <PropertyGroup>
    <TargetFramework>net10.0</TargetFramework>
    <Nullable>enable</Nullable>
    <ImplicitUsings>enable</ImplicitUsings>
    <RootNamespace>C2.Api</RootNamespace>
  </PropertyGroup>

  <ItemGroup>
    <PackageReference Include="Grpc.AspNetCore.Server.ClientFactory" Version="2.66.0" />
    <PackageReference Include="Grpc.Net.Client" Version="2.66.0" />
    <PackageReference Include="Grpc.Tools" Version="2.66.0">
      <PrivateAssets>all</PrivateAssets>
    </PackageReference>
    <PackageReference Include="Google.Protobuf" Version="3.28.2" />
    <PackageReference Include="Microsoft.AspNetCore.SignalR" Version="1.2.0" />
    <PackageReference Include="Microsoft.EntityFrameworkCore" Version="10.0.0" />
    <PackageReference Include="Microsoft.EntityFrameworkCore.Design" Version="10.0.0" />
    <PackageReference Include="Npgsql.EntityFrameworkCore.PostgreSQL" Version="10.0.0" />
  </ItemGroup>

</Project>
```

> If a package version above isn't available at install time, use `dotnet add package <name>` and let NuGet pick the latest matching .NET 10. The version pins above are guidance, not gospel.

- [ ] **Step 2: Create `webapp/api/Program.cs`** (stub — full version in Task 8)

```csharp
var builder = WebApplication.CreateBuilder(args);

var app = builder.Build();

app.MapGet("/health", () => "ok");

app.Run();
```

- [ ] **Step 3: Create `webapp/api/Dockerfile`**

```dockerfile
# Build stage
FROM mcr.microsoft.com/dotnet/sdk:10.0 AS build
WORKDIR /src

COPY Api.csproj ./
RUN dotnet restore Api.csproj

COPY . ./
RUN dotnet publish Api.csproj -c Release -o /app /p:UseAppHost=false

# Runtime stage
FROM mcr.microsoft.com/dotnet/aspnet:10.0
WORKDIR /app
COPY --from=build /app ./
EXPOSE 8080
ENTRYPOINT ["dotnet", "Api.dll"]
```

- [ ] **Step 4: Create `webapp/api/.dockerignore`**

```
bin/
obj/
*.user
```

- [ ] **Step 5: Build the API image**

Run: `cd webapp && docker compose build api`
Expected: image builds. Restore + publish should finish in 1-3 min.

- [ ] **Step 6: Verify health endpoint**

Run: `cd webapp && docker compose up -d api postgres`
Run: `curl http://localhost:8080/health`
Expected: `ok`

Then: `docker compose down`

- [ ] **Step 7: Commit**

```bash
git add webapp/api/
git commit -m "feat(c2-ui): scaffold .NET 10 minimal API with health endpoint"
```

---

## Task 5: gRPC stub generation from SCRIMMAGE protos

**Files:**
- Create: `webapp/api/Protos/Scrimmage.proto`
- Create: `webapp/api/Protos/Frame.proto`
- Create: `webapp/api/Protos/Contact.proto`
- Create: `webapp/api/Protos/State.proto`
- Create: `webapp/api/Protos/ID.proto`
- Create: `webapp/api/Protos/Vector3d.proto`
- Create: `webapp/api/Protos/Quaternion.proto`
- Create: any other `.proto` files referenced transitively (UTMTerrain, ContactVisual, Shape, GUIMsg, SimInfo, WorldPointClicked) — copy them all from `src/proto/scrimmage/proto/` to be safe
- Modify: `webapp/api/Api.csproj`

- [ ] **Step 1: Copy all `.proto` files from SCRIMMAGE source tree**

Run (from repo root): `cp -r src/proto/scrimmage/proto webapp/api/Protos`
Expected: `webapp/api/Protos/` now contains `Scrimmage.proto`, `Frame.proto`, `Contact.proto`, `State.proto`, `ID.proto`, `Vector3d.proto`, `Quaternion.proto`, and several others.

- [ ] **Step 2: Update `webapp/api/Api.csproj` to compile the protos**

Add inside the `<Project>` element after the existing `<ItemGroup>`:

```xml
  <ItemGroup>
    <Protobuf Include="Protos\**\*.proto" GrpcServices="Client" ProtoRoot="Protos" />
  </ItemGroup>
```

> The `ProtoRoot="Protos"` is important: SCRIMMAGE's protos use imports like `import "scrimmage/proto/Contact.proto";`, so the include root must allow that path. Adjust folder layout if needed — the safest approach is `webapp/api/Protos/scrimmage/proto/*.proto` mirroring the SCRIMMAGE layout exactly. If `Protobuf Include="Protos\**\*.proto"` with the mirrored layout doesn't resolve imports, fall back to placing all protos at `Protos/` root and rewriting `import` paths.

- [ ] **Step 3: Verify protos compile**

Run: `cd webapp && docker compose build api`
Expected: build succeeds. Generated C# clients (`ScrimmageService.ScrimmageServiceClient`, `Frame`, `Contact`, etc.) are now available in the `Scrimmage_Proto` namespace.

> **If you get import errors:** the cleanest fix is to copy protos into `Protos/scrimmage/proto/` so the import paths resolve as written. Then `<Protobuf Include="Protos\scrimmage\proto\*.proto" GrpcServices="Client" ProtoRoot="Protos" />`.

- [ ] **Step 4: Commit**

```bash
git add webapp/api/
git commit -m "feat(c2-ui): wire SCRIMMAGE proto stubs into API project"
```

---

## Task 6: Host ScrimmageService gRPC server + DTO mapping

The API **hosts** the `ScrimmageService` gRPC server; the SCRIMMAGE process is the gRPC client that pushes `Frame` messages to it (this is how SCRIMMAGE's native viewer also works — the viewer is the server, scrimmage is the client). Frames are mapped to a JSON DTO and broadcast to all SignalR subscribers. We also patch the launcher in this task to rewrite `<stream_ip>` and `<stream_port>` so scrimmage targets the `api` container.

**Files:**
- Create: `webapp/api/Dtos.cs`
- Create: `webapp/api/FrameStreamService.cs`
- Modify: `webapp/api/Program.cs`
- Modify: `webapp/scrimmage-runner/launcher/app.py`

- [ ] **Step 1: Create `webapp/api/Dtos.cs`**

```csharp
namespace C2.Api;

public record Vec3(double X, double Y, double Z);
public record Quat(double W, double X, double Y, double Z);

public record EntityDto(
    int Id,
    int TeamId,
    int SubSwarmId,
    string Type,
    bool Active,
    Vec3 Position,
    Vec3 Velocity,
    Quat Orientation
);

public record FrameDto(double Time, IReadOnlyList<EntityDto> Entities);

public record OriginDto(double Lat, double Lon, double Alt);

public record StartMissionRequest(string Name);

public record MissionStartResponse(string Status, int Pid, string Mission, OriginDto Origin);

public record StatusResponse(string Status, string? Mission, double? UptimeS, OriginDto? Origin);
```

- [ ] **Step 2: Create `webapp/api/FrameStreamService.cs`**

```csharp
using Grpc.Core;
using Microsoft.AspNetCore.SignalR;
using Scrimmage_Proto;

namespace C2.Api;

public class FrameStreamService : ScrimmageService.ScrimmageServiceBase
{
    private readonly ILogger<FrameStreamService> _log;
    private readonly IHubContext<FrameHub> _hub;

    public FrameStreamService(ILogger<FrameStreamService> log, IHubContext<FrameHub> hub)
    {
        _log = log;
        _hub = hub;
    }

    public override async Task<BlankReply> SendFrame(
        IAsyncStreamReader<Frame> requestStream,
        ServerCallContext context)
    {
        _log.LogInformation("Frame stream opened from {peer}", context.Peer);
        await foreach (var frame in requestStream.ReadAllAsync(context.CancellationToken))
        {
            var dto = MapFrame(frame);
            await _hub.Clients.All.SendAsync("OnFrame", dto, context.CancellationToken);
        }
        _log.LogInformation("Frame stream closed from {peer}", context.Peer);
        return new BlankReply();
    }

    // SCRIMMAGE also calls SendUTMTerrain, SendShapes, SendContactVisual, SendSimInfo,
    // SendGUIMsg, SendWorldPointClicked. Implement as no-ops so the service contract is
    // satisfied — frames are the only thing we consume in the MVP.
    public override Task<BlankReply> SendUTMTerrain(UTMTerrain request, ServerCallContext ctx)
        => Task.FromResult(new BlankReply());
    public override Task<BlankReply> SendShapes(Shapes request, ServerCallContext ctx)
        => Task.FromResult(new BlankReply());
    public override Task<BlankReply> SendContactVisual(ContactVisual request, ServerCallContext ctx)
        => Task.FromResult(new BlankReply());
    public override Task<BlankReply> SendSimInfo(SimInfo request, ServerCallContext ctx)
        => Task.FromResult(new BlankReply());
    public override Task<BlankReply> SendGUIMsg(GUIMsg request, ServerCallContext ctx)
        => Task.FromResult(new BlankReply());
    public override Task<SimCommand> SendWorldPointClicked(WorldPointClicked request, ServerCallContext ctx)
        => Task.FromResult(new SimCommand());

    private static FrameDto MapFrame(Frame f)
    {
        var entities = new List<EntityDto>(f.Contact.Count);
        foreach (var c in f.Contact)
        {
            entities.Add(new EntityDto(
                Id: c.Id?.Id ?? 0,
                TeamId: c.Id?.TeamId ?? 0,
                SubSwarmId: c.Id?.SubSwarmId ?? 0,
                Type: c.Type.ToString(),
                Active: c.Active,
                Position: new Vec3(c.State?.Position?.X ?? 0, c.State?.Position?.Y ?? 0, c.State?.Position?.Z ?? 0),
                Velocity: new Vec3(c.State?.LinearVelocity?.X ?? 0, c.State?.LinearVelocity?.Y ?? 0, c.State?.LinearVelocity?.Z ?? 0),
                Orientation: new Quat(c.State?.Orientation?.W ?? 1, c.State?.Orientation?.X ?? 0, c.State?.Orientation?.Y ?? 0, c.State?.Orientation?.Z ?? 0)
            ));
        }
        return new FrameDto(f.Time, entities);
    }
}
```

> **Proto-stub uncertainty to resolve at this step:** the exact method names + request/response types above (`BlankReply`, `SimCommand`, `WorldPointClicked`, etc.) come from inspecting `Scrimmage.proto` during design. Open `webapp/api/Protos/Scrimmage.proto` (or `webapp/api/Protos/scrimmage/proto/Scrimmage.proto`) and adjust the override signatures if the generated `ScrimmageServiceBase` differs. The `SendFrame` shape (client-streaming `Frame` → unary reply) is the load-bearing one; the no-op overrides exist only to satisfy the abstract base class. If a method in the base class isn't overridden, gRPC returns `Unimplemented` to the caller, which scrimmage may treat as a fatal error — so all base methods must be present.

- [ ] **Step 3: Replace `webapp/api/Program.cs`**

```csharp
using C2.Api;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddGrpc();
builder.Services.AddSignalR();
builder.Services.AddCors(o => o.AddDefaultPolicy(p =>
    p.WithOrigins("http://localhost:5173").AllowAnyHeader().AllowAnyMethod().AllowCredentials()));

// Kestrel serves both gRPC (HTTP/2 cleartext) and HTTP/1.1 (REST + SignalR) on one port
builder.WebHost.ConfigureKestrel(opts =>
{
    opts.ListenAnyIP(8080, l => l.Protocols = Microsoft.AspNetCore.Server.Kestrel.Core.HttpProtocols.Http1AndHttp2);
});

var app = builder.Build();

app.UseCors();
app.MapGrpcService<FrameStreamService>();
app.MapHub<FrameHub>("/hubs/frames");
app.MapGet("/health", () => "ok");

app.Run();
```

- [ ] **Step 4: Update launcher to rewrite `<stream_ip>` and `<stream_port>`**

In `webapp/scrimmage-runner/launcher/app.py`, inside `_template_mission`, **after** the `enable_gui` regex substitutions and **before** `ACTIVE_MISSION_PATH.write_text(text)`, add:

```python
    # Route the scrimmage gRPC stream to the API container instead of localhost
    text = re.sub(r'<stream_ip>[^<]*</stream_ip>', '<stream_ip>api</stream_ip>', text)
    text = re.sub(r'<stream_port>[^<]*</stream_port>', '<stream_port>8080</stream_port>', text)
```

> Why port 8080: scrimmage speaks HTTP/2 cleartext gRPC to the API on the same port the API uses for everything else. Kestrel multiplexes HTTP/1.1 and HTTP/2 on one port (configured in Step 3).

- [ ] **Step 5: Build the API + scrimmage-runner**

Run: `cd webapp && docker compose build api scrimmage`
Expected: both build successfully. The `webapp/api/Protos/` files compile without errors (Task 5 already verified).

> If the build fails inside `FrameStreamService.cs` on a method signature, open the generated `ScrimmageServiceBase` (right-click → Go to Definition in an IDE, or look under `obj/Debug/net10.0/` for the generated `.cs` files) and align the override signatures.

- [ ] **Step 6: Commit**

```bash
git add webapp/
git commit -m "feat(c2-ui): host ScrimmageService gRPC server, map frames to SignalR DTO, route scrimmage to API"
```

---

## Task 7: SignalR FrameHub

**Files:**
- Create: `webapp/api/FrameHub.cs`

- [ ] **Step 1: Create `webapp/api/FrameHub.cs`**

```csharp
using Microsoft.AspNetCore.SignalR;

namespace C2.Api;

public class FrameHub : Hub
{
    // No methods needed — clients only subscribe; FrameStreamService broadcasts via IHubContext.
}
```

- [ ] **Step 2: Confirm `Program.cs` already maps the hub**

(`app.MapHub<FrameHub>("/hubs/frames");` from Task 6 Step 3 — no change needed.)

- [ ] **Step 3: Build**

Run: `cd webapp && docker compose build api`
Expected: success.

- [ ] **Step 4: Commit**

```bash
git add webapp/api/FrameHub.cs
git commit -m "feat(c2-ui): add SignalR FrameHub for frame fan-out"
```

---

## Task 8: REST endpoints — mission list / start / stop / status

**Files:**
- Create: `webapp/api/Endpoints.cs`
- Modify: `webapp/api/Program.cs`

- [ ] **Step 1: Create `webapp/api/Endpoints.cs`**

```csharp
using System.Net.Http.Json;

namespace C2.Api;

public static class Endpoints
{
    public static void MapMissionEndpoints(this WebApplication app)
    {
        var launcher = app.Configuration["SCRIMMAGE_LAUNCHER_URL"] ?? "http://scrimmage:5050";

        app.MapGet("/api/missions", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/missions");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/start", async (
            StartMissionRequest req,
            IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsJsonAsync($"{launcher}/missions/start", req);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapPost("/api/missions/stop", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.PostAsync($"{launcher}/missions/stop", null);
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });

        app.MapGet("/api/status", async (IHttpClientFactory http) =>
        {
            var client = http.CreateClient();
            var resp = await client.GetAsync($"{launcher}/status");
            var body = await resp.Content.ReadAsStringAsync();
            return Results.Content(body, "application/json", statusCode: (int)resp.StatusCode);
        });
    }
}
```

- [ ] **Step 2: Update `webapp/api/Program.cs` to register HttpClient + the endpoints**

Add after `builder.Services.AddCors(...);`:

```csharp
builder.Services.AddHttpClient();
```

And after `app.MapGet("/health", () => "ok");`:

```csharp
app.MapMissionEndpoints();
```

- [ ] **Step 3: Configure JSON serializer to use camelCase consistently**

System.Text.Json defaults in ASP.NET Core 6+ already produce camelCase JSON for record types (`Id` → `"id"`). Verify by spot-checking the response in Task 11. No change needed unless that fails.

- [ ] **Step 4: Build**

Run: `cd webapp && docker compose build api`
Expected: success.

- [ ] **Step 5: Commit**

```bash
git add webapp/api/
git commit -m "feat(c2-ui): add REST endpoints for mission lifecycle (proxy to launcher)"
```

---

## Task 9: EF Core + Postgres scaffolding (no entities)

**Files:**
- Create: `webapp/api/AppDbContext.cs`
- Modify: `webapp/api/Program.cs`

- [ ] **Step 1: Create `webapp/api/AppDbContext.cs`**

```csharp
using Microsoft.EntityFrameworkCore;

namespace C2.Api;

public class AppDbContext : DbContext
{
    public AppDbContext(DbContextOptions<AppDbContext> options) : base(options) { }

    // No entities yet — added in post-MVP iterations (commands, audit log, tags).
}
```

- [ ] **Step 2: Register the DbContext in `Program.cs`**

After `builder.Services.AddHttpClient();` add:

```csharp
builder.Services.AddDbContext<AppDbContext>(opts =>
    opts.UseNpgsql(builder.Configuration.GetConnectionString("Default")));
```

- [ ] **Step 3: Build**

Run: `cd webapp && docker compose build api`
Expected: success.

- [ ] **Step 4: Commit**

```bash
git add webapp/api/
git commit -m "feat(c2-ui): scaffold EF Core + Postgres DbContext (no entities yet)"
```

---

## Task 10: Verify API end-to-end with scrimmage running

Checkpoint task. We bring up scrimmage + api + postgres, start a mission, watch the API logs to confirm frames are received and broadcast.

**Files:** none (verification only)

- [ ] **Step 1: Bring up scrimmage + api + postgres**

Run: `cd webapp && docker compose up scrimmage api postgres`
Expected: all three start; API logs `Now listening on: http://[::]:8080`.

- [ ] **Step 2: List missions through the API**

In another terminal: `curl http://localhost:8080/api/missions`
Expected: same JSON array as the launcher returned in Task 3.

- [ ] **Step 3: Start capture-the-flag through the API**

```bash
curl -X POST http://localhost:8080/api/missions/start \
  -H "Content-Type: application/json" \
  -d '{"name":"capture-the-flag.xml"}'
```

Expected: JSON with status, pid, mission name, and origin (lat 35.72…, lon -120.76…, alt 300).

- [ ] **Step 4: Watch API logs for `Frame stream opened from …`**

Within ~3 seconds of start, the API logs should show "Frame stream opened from …" (the SCRIMMAGE container's IP on the c2net network). If you don't see this within 10 seconds, **stop and debug** — see "If frames don't flow" below.

- [ ] **Step 5: Verify SignalR by connecting a quick test client**

Easiest path: install `wscat` or use the browser console:

```javascript
// In any browser at any page (or use a tool like Postman):
const c = new signalR.HubConnectionBuilder().withUrl("http://localhost:8080/hubs/frames").build();
c.on("OnFrame", f => console.log(f.time, f.entities.length));
await c.start();
```

Expected: console logs `time=N entities=20` (or however many) repeatedly.

> **If you don't have signalR.js loaded:** the JS client also exists as a CDN: `<script src="https://cdn.jsdelivr.net/npm/@microsoft/signalr@8/dist/browser/signalr.min.js"></script>`. Or skip this verification and confirm in the next phase when the React client connects.

- [ ] **Step 6: Stop the mission**

```bash
curl -X POST http://localhost:8080/api/missions/stop
```

Expected: API logs "Frame stream closed". Restart same mission to confirm the API reconnects cleanly.

- [ ] **Step 7: Tear down + commit checkpoint**

```bash
docker compose down
git add -A
git commit -m "fix(c2-ui): API end-to-end smoke-test fixes" --allow-empty
```

> **If frames don't flow:** check (a) the launcher actually rewrote `<stream_ip>` and `<stream_port>` (cat `/tmp/active_mission.xml` inside the scrimmage container after starting a mission), (b) Kestrel is listening on HTTP/2 not just HTTP/1 (look for "HTTP/2" in the API startup logs), (c) the SCRIMMAGE process can resolve `api` as a hostname (it's on the same compose network so this should work). Fix and re-run before moving to the web work.

---

## Task 11: Web project scaffold + Dockerfile

**Files:**
- Create: `webapp/web/package.json`
- Create: `webapp/web/tsconfig.json`
- Create: `webapp/web/vite.config.ts`
- Create: `webapp/web/index.html`
- Create: `webapp/web/src/main.tsx`
- Create: `webapp/web/src/AppShell.tsx` (stub — fleshed out in Task 18)
- Create: `webapp/web/Dockerfile`
- Create: `webapp/web/.dockerignore`
- Create: `webapp/web/.env.example`

- [ ] **Step 1: Create `webapp/web/package.json`**

```json
{
  "name": "c2-web",
  "private": true,
  "version": "0.1.0",
  "type": "module",
  "scripts": {
    "dev": "vite --host 0.0.0.0",
    "build": "tsc && vite build",
    "preview": "vite preview --host 0.0.0.0",
    "test": "vitest run"
  },
  "dependencies": {
    "@microsoft/signalr": "^8.0.7",
    "cesium": "^1.121.0",
    "react": "^18.3.1",
    "react-dom": "^18.3.1"
  },
  "devDependencies": {
    "@types/react": "^18.3.12",
    "@types/react-dom": "^18.3.1",
    "@vitejs/plugin-react": "^4.3.3",
    "typescript": "^5.6.3",
    "vite": "^5.4.10",
    "vite-plugin-cesium": "^1.2.23",
    "vitest": "^2.1.4"
  }
}
```

- [ ] **Step 2: Create `webapp/web/tsconfig.json`**

```json
{
  "compilerOptions": {
    "target": "ES2022",
    "useDefineForClassFields": true,
    "lib": ["ES2022", "DOM", "DOM.Iterable"],
    "module": "ESNext",
    "skipLibCheck": true,
    "moduleResolution": "bundler",
    "allowImportingTsExtensions": false,
    "isolatedModules": true,
    "moduleDetection": "force",
    "noEmit": true,
    "jsx": "react-jsx",
    "strict": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "noFallthroughCasesInSwitch": true
  },
  "include": ["src"]
}
```

- [ ] **Step 3: Create `webapp/web/vite.config.ts`**

```ts
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import cesium from 'vite-plugin-cesium';

export default defineConfig({
  plugins: [react(), cesium()],
  server: { port: 5173, host: '0.0.0.0' }
});
```

- [ ] **Step 4: Create `webapp/web/index.html`**

```html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <title>SCRIMMAGE C2</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  </head>
  <body style="margin:0">
    <div id="root"></div>
    <script type="module" src="/src/main.tsx"></script>
  </body>
</html>
```

- [ ] **Step 5: Create `webapp/web/src/main.tsx`**

```tsx
import React from 'react';
import ReactDOM from 'react-dom/client';
import { AppShell } from './AppShell';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <AppShell />
  </React.StrictMode>
);
```

- [ ] **Step 6: Create `webapp/web/src/AppShell.tsx`** (stub — Task 18 fleshes out the layout)

```tsx
export function AppShell() {
  return <div style={{ padding: 16 }}>SCRIMMAGE C2 — UI scaffold (Task 11)</div>;
}
```

- [ ] **Step 7: Create `webapp/web/Dockerfile`** (Vite dev server with hot reload)

```dockerfile
FROM node:20-alpine
WORKDIR /app

COPY package.json ./
RUN npm install

COPY . ./

EXPOSE 5173
CMD ["npm", "run", "dev"]
```

- [ ] **Step 8: Create `webapp/web/.dockerignore`**

```
node_modules/
dist/
```

- [ ] **Step 9: Create `webapp/web/.env.example`**

```
VITE_API_URL=http://localhost:8080
VITE_CESIUM_ION_TOKEN=your_token_here
```

- [ ] **Step 10: Build + smoke test**

Run: `cd webapp && docker compose up --build web`
Browser → http://localhost:5173
Expected: page shows "SCRIMMAGE C2 — UI scaffold (Task 11)".

`docker compose down` to clean up.

- [ ] **Step 11: Commit**

```bash
git add webapp/web/
git commit -m "feat(c2-ui): scaffold React + Vite + TypeScript web project"
```

---

## Task 12: Cesium base viewer

**Files:**
- Create: `webapp/web/src/components/CesiumViewer.tsx`
- Modify: `webapp/web/src/AppShell.tsx`

- [ ] **Step 1: Create `webapp/web/src/components/CesiumViewer.tsx`** (basic viewer over Camp Roberts)

```tsx
import { useEffect, useRef } from 'react';
import * as Cesium from 'cesium';

const ION_TOKEN = (import.meta as any).env.VITE_CESIUM_ION_TOKEN as string | undefined;

export interface ViewerProps {
  origin?: { lat: number; lon: number; alt: number };
}

export function CesiumViewer({ origin }: ViewerProps) {
  const ref = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Cesium.Viewer | null>(null);

  useEffect(() => {
    if (!ref.current) return;
    if (ION_TOKEN) Cesium.Ion.defaultAccessToken = ION_TOKEN;

    const viewer = new Cesium.Viewer(ref.current, {
      timeline: false,
      animation: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
      terrainProvider: undefined, // set after token loads, below
    });
    viewerRef.current = viewer;

    // Try to load Ion world terrain; fall back silently if no token
    if (ION_TOKEN) {
      Cesium.createWorldTerrainAsync()
        .then(t => { viewer.terrainProvider = t; })
        .catch(err => console.warn('Cesium terrain load failed:', err));
    }

    // Default camera position: Camp Roberts area (until origin arrives)
    const defaultLat = origin?.lat ?? 35.721025;
    const defaultLon = origin?.lon ?? -120.767925;
    const defaultAlt = (origin?.alt ?? 300) + 2000;
    viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(defaultLon, defaultLat, defaultAlt),
    });

    return () => { viewer.destroy(); viewerRef.current = null; };
  }, [origin?.lat, origin?.lon, origin?.alt]);

  return <div ref={ref} style={{ width: '100%', height: '100%' }} />;
}
```

- [ ] **Step 2: Update `webapp/web/src/AppShell.tsx` to render the viewer**

```tsx
import { CesiumViewer } from './components/CesiumViewer';

export function AppShell() {
  return (
    <div style={{ position: 'absolute', inset: 0 }}>
      <CesiumViewer />
    </div>
  );
}
```

- [ ] **Step 3: Set up local `.env` with your Cesium token**

In `webapp/web/`: `cp .env.example .env`, then edit `.env` to include your real Ion token. The compose env-passthrough in Task 1 picks it up via `${CESIUM_ION_TOKEN}` from `webapp/.env`. Also create/edit `webapp/.env` with `CESIUM_ION_TOKEN=...`.

- [ ] **Step 4: Smoke test**

Run: `cd webapp && docker compose up --build web`
Browser → http://localhost:5173
Expected: 3D Earth renders, camera positioned over Camp Roberts area, terrain loaded if token is valid.

- [ ] **Step 5: Commit**

```bash
git add webapp/web/
git commit -m "feat(c2-ui): add Cesium viewer with default camera over Camp Roberts"
```

---

## Task 13: enuToCartesian utility — TDD

This is the **one unit test** in the MVP. The conversion is the only piece of nontrivial math; a bug puts drones in the wrong place silently.

**Files:**
- Create: `webapp/web/src/lib/enuToCartesian.ts`
- Create: `webapp/web/src/lib/enuToCartesian.test.ts`

- [ ] **Step 1: Write the failing test** (`webapp/web/src/lib/enuToCartesian.test.ts`)

```ts
import { describe, expect, it } from 'vitest';
import * as Cesium from 'cesium';
import { enuToCartesian } from './enuToCartesian';

describe('enuToCartesian', () => {
  const origin = { lat: 35.721025, lon: -120.767925, alt: 300 };

  it('returns the origin position when local offset is (0,0,0)', () => {
    const result = enuToCartesian(origin, 0, 0, 0);
    const expected = Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt);
    expect(result.x).toBeCloseTo(expected.x, 1);
    expect(result.y).toBeCloseTo(expected.y, 1);
    expect(result.z).toBeCloseTo(expected.z, 1);
  });

  it('moves north (positive y) by ~111 km per degree of latitude', () => {
    // 1 degree latitude ≈ 111,320 m. We move 1000 m north and check the latitude delta.
    const result = enuToCartesian(origin, 0, 1000, 0);
    const carto = Cesium.Cartographic.fromCartesian(result);
    const latDeg = Cesium.Math.toDegrees(carto.latitude);
    const expectedLatDeg = origin.lat + 1000 / 111320;
    expect(latDeg).toBeCloseTo(expectedLatDeg, 4);
  });

  it('moves east (positive x) and stays close to origin altitude', () => {
    const result = enuToCartesian(origin, 1000, 0, 0);
    const carto = Cesium.Cartographic.fromCartesian(result);
    expect(carto.height).toBeCloseTo(origin.alt, 0); // within 1 m
  });
});
```

- [ ] **Step 2: Run the test to confirm it fails**

Run: `cd webapp/web && npm install && npm test`
Expected: FAIL with "Cannot find module './enuToCartesian'" or similar.

- [ ] **Step 3: Implement** (`webapp/web/src/lib/enuToCartesian.ts`)

```ts
import * as Cesium from 'cesium';

export interface Origin { lat: number; lon: number; alt: number; }

/**
 * Convert a local ENU offset (meters) from a geographic origin to a Cartesian3 in ECEF.
 * SCRIMMAGE positions are reported as ENU offsets from the mission's lat/lon/alt origin.
 */
export function enuToCartesian(
  origin: Origin,
  east: number,
  north: number,
  up: number
): Cesium.Cartesian3 {
  const originEcef = Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt);
  const enuFrame = Cesium.Transforms.eastNorthUpToFixedFrame(originEcef);
  const offset = new Cesium.Cartesian3(east, north, up);
  const result = new Cesium.Cartesian3();
  Cesium.Matrix4.multiplyByPoint(enuFrame, offset, result);
  return result;
}
```

- [ ] **Step 4: Run the test to confirm it passes**

Run: `npm test`
Expected: all three tests PASS.

- [ ] **Step 5: Commit**

```bash
git add webapp/web/src/lib/
git commit -m "feat(c2-ui): add enuToCartesian util with unit tests"
```

---

## Task 14: SignalR client hook (useFrameStream)

**Files:**
- Create: `webapp/web/src/types.ts`
- Create: `webapp/web/src/hooks/useFrameStream.ts`

- [ ] **Step 1: Create `webapp/web/src/types.ts`** (mirror of API DTOs)

```ts
export interface Vec3 { x: number; y: number; z: number; }
export interface Quat { w: number; x: number; y: number; z: number; }

export interface EntityDto {
  id: number;
  teamId: number;
  subSwarmId: number;
  type: string;
  active: boolean;
  position: Vec3;
  velocity: Vec3;
  orientation: Quat;
}

export interface FrameDto {
  time: number;
  entities: EntityDto[];
}

export interface Origin { lat: number; lon: number; alt: number; }

export interface MissionStartResponse {
  status: string;
  pid: number;
  mission: string;
  origin: Origin;
}
```

- [ ] **Step 2: Create `webapp/web/src/hooks/useFrameStream.ts`**

The hook intentionally does NOT trigger a React re-render per frame. It writes the latest frame to a ref + invokes a callback so Cesium can update its scene graph imperatively. The sidebar entity list (Task 18) reads via a throttled `useState` update.

```ts
import * as signalR from '@microsoft/signalr';
import { useEffect, useRef, useState } from 'react';
import type { FrameDto } from '../types';

const API_URL = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

type FrameHandler = (frame: FrameDto) => void;

/**
 * Subscribes to /hubs/frames. The `onFrame` callback fires on every frame (no React re-render).
 * Also exposes a throttled `latestFrame` state that updates ~5 Hz, suitable for sidebar UI.
 */
export function useFrameStream(onFrame: FrameHandler) {
  const [connected, setConnected] = useState(false);
  const [latestFrame, setLatestFrame] = useState<FrameDto | null>(null);
  const lastUiUpdate = useRef(0);
  const handlerRef = useRef(onFrame);
  handlerRef.current = onFrame;

  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder()
      .withUrl(`${API_URL}/hubs/frames`)
      .withAutomaticReconnect()
      .build();

    conn.on('OnFrame', (frame: FrameDto) => {
      handlerRef.current(frame);
      const now = performance.now();
      if (now - lastUiUpdate.current > 200) {
        lastUiUpdate.current = now;
        setLatestFrame(frame);
      }
    });

    conn.onreconnected(() => setConnected(true));
    conn.onclose(() => setConnected(false));

    conn.start()
      .then(() => setConnected(true))
      .catch(err => console.error('SignalR connect failed:', err));

    return () => { conn.stop(); };
  }, []);

  return { connected, latestFrame };
}
```

- [ ] **Step 3: Build to verify compile**

Run: `cd webapp/web && npx tsc --noEmit`
Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add webapp/web/src/
git commit -m "feat(c2-ui): add useFrameStream SignalR hook"
```

---

## Task 15: CesiumViewer renders entities from frames

**Files:**
- Modify: `webapp/web/src/components/CesiumViewer.tsx`

- [ ] **Step 1: Replace `webapp/web/src/components/CesiumViewer.tsx`** with the version that consumes frames

```tsx
import { useEffect, useRef } from 'react';
import * as Cesium from 'cesium';
import type { FrameDto, Origin } from '../types';
import { enuToCartesian } from '../lib/enuToCartesian';

const ION_TOKEN = (import.meta as any).env.VITE_CESIUM_ION_TOKEN as string | undefined;

export interface ViewerHandle {
  applyFrame: (frame: FrameDto) => void;
  setOrigin: (origin: Origin | null) => void;
}

export interface ViewerProps {
  onReady?: (handle: ViewerHandle) => void;
}

const TEAM_COLORS: Record<number, Cesium.Color> = {
  1: Cesium.Color.DODGERBLUE,
  2: Cesium.Color.CRIMSON,
};

export function CesiumViewer({ onReady }: ViewerProps) {
  const ref = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Cesium.Viewer | null>(null);
  const entitiesRef = useRef<Map<number, Cesium.Entity>>(new Map());
  const originRef = useRef<Origin | null>(null);

  useEffect(() => {
    if (!ref.current) return;
    if (ION_TOKEN) Cesium.Ion.defaultAccessToken = ION_TOKEN;

    const viewer = new Cesium.Viewer(ref.current, {
      timeline: false,
      animation: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
    });
    viewerRef.current = viewer;

    if (ION_TOKEN) {
      Cesium.createWorldTerrainAsync()
        .then(t => { viewer.terrainProvider = t; })
        .catch(err => console.warn('Cesium terrain load failed:', err));
    }

    // Default view (overwritten when origin is set)
    viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(-120.767925, 35.721025, 3000),
    });

    const handle: ViewerHandle = {
      applyFrame: (frame) => applyFrame(frame, viewer, entitiesRef.current, originRef.current),
      setOrigin: (origin) => {
        originRef.current = origin;
        if (origin) {
          viewer.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt + 2500),
            duration: 1.5,
          });
        }
      },
    };
    onReady?.(handle);

    return () => { viewer.destroy(); viewerRef.current = null; entitiesRef.current.clear(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <div ref={ref} style={{ width: '100%', height: '100%' }} />;
}

function applyFrame(
  frame: FrameDto,
  viewer: Cesium.Viewer,
  entities: Map<number, Cesium.Entity>,
  origin: Origin | null
) {
  if (!origin) return; // can't render without origin

  const seen = new Set<number>();
  for (const e of frame.entities) {
    seen.add(e.id);
    if (!e.active) {
      const existing = entities.get(e.id);
      if (existing) { viewer.entities.remove(existing); entities.delete(e.id); }
      continue;
    }
    const pos = enuToCartesian(origin, e.position.x, e.position.y, e.position.z);
    const color = TEAM_COLORS[e.teamId] ?? Cesium.Color.GRAY;
    let ent = entities.get(e.id);
    if (!ent) {
      ent = viewer.entities.add({
        position: pos,
        point: { pixelSize: 12, color, outlineColor: Cesium.Color.WHITE, outlineWidth: 2 },
        label: {
          text: `#${e.id}`,
          font: '12px sans-serif',
          pixelOffset: new Cesium.Cartesian2(0, -20),
          fillColor: Cesium.Color.WHITE,
          showBackground: true,
          backgroundColor: Cesium.Color.BLACK.withAlpha(0.6),
        },
      });
      entities.set(e.id, ent);
    } else {
      ent.position = new Cesium.ConstantPositionProperty(pos);
    }
  }

  // Remove entities that disappeared from the frame entirely
  for (const [id, ent] of entities) {
    if (!seen.has(id)) { viewer.entities.remove(ent); entities.delete(id); }
  }
}
```

- [ ] **Step 2: Build to verify compile**

Run: `cd webapp/web && npx tsc --noEmit`
Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add webapp/web/src/components/CesiumViewer.tsx
git commit -m "feat(c2-ui): render frame entities as Cesium points colored by team"
```

---

## Task 16: MissionPicker component

**Files:**
- Create: `webapp/web/src/components/MissionPicker.tsx`
- Create: `webapp/web/src/lib/api.ts`

- [ ] **Step 1: Create `webapp/web/src/lib/api.ts`** (thin REST wrapper)

```ts
import type { MissionStartResponse } from '../types';

const BASE = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

export async function listMissions(): Promise<string[]> {
  const r = await fetch(`${BASE}/api/missions`);
  if (!r.ok) throw new Error(`listMissions: ${r.status}`);
  return r.json();
}

export async function startMission(name: string): Promise<MissionStartResponse> {
  const r = await fetch(`${BASE}/api/missions/start`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  });
  if (!r.ok) {
    const err = await r.text();
    throw new Error(`startMission failed: ${r.status} ${err}`);
  }
  return r.json();
}

export async function stopMission(): Promise<void> {
  const r = await fetch(`${BASE}/api/missions/stop`, { method: 'POST' });
  if (!r.ok) throw new Error(`stopMission: ${r.status}`);
}
```

- [ ] **Step 2: Create `webapp/web/src/components/MissionPicker.tsx`**

```tsx
import { useEffect, useState } from 'react';
import type { MissionStartResponse } from '../types';
import { listMissions, startMission, stopMission } from '../lib/api';

export interface MissionPickerProps {
  onStarted: (resp: MissionStartResponse) => void;
  onStopped: () => void;
}

export function MissionPicker({ onStarted, onStopped }: MissionPickerProps) {
  const [missions, setMissions] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>('');
  const [running, setRunning] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    listMissions()
      .then(list => {
        setMissions(list);
        const preferred = list.find(m => m === 'capture-the-flag.xml') ?? list[0] ?? '';
        setSelected(preferred);
      })
      .catch(e => setError(String(e)));
  }, []);

  async function handleStart() {
    if (!selected) return;
    setBusy(true); setError(null);
    try {
      const resp = await startMission(selected);
      setRunning(selected);
      onStarted(resp);
    } catch (e) {
      setError(String(e));
    } finally { setBusy(false); }
  }

  async function handleStop() {
    setBusy(true); setError(null);
    try {
      await stopMission();
      setRunning(null);
      onStopped();
    } catch (e) {
      setError(String(e));
    } finally { setBusy(false); }
  }

  return (
    <div style={{ display: 'flex', gap: 8, alignItems: 'center', padding: 8, background: '#1a1a1a', color: '#eee' }}>
      <label>Mission:</label>
      <select
        value={selected}
        onChange={e => setSelected(e.target.value)}
        disabled={busy || running !== null}
        style={{ padding: 4 }}
      >
        {missions.map(m => <option key={m} value={m}>{m}</option>)}
      </select>
      <button onClick={handleStart} disabled={busy || running !== null || !selected}>Start</button>
      <button onClick={handleStop} disabled={busy || running === null}>Stop</button>
      <span style={{ marginLeft: 16, color: running ? '#7f7' : '#999' }}>
        {running ? `Running: ${running}` : 'Idle'}
      </span>
      {error && <span style={{ marginLeft: 16, color: '#f77' }}>{error}</span>}
    </div>
  );
}
```

- [ ] **Step 3: Build to verify compile**

Run: `cd webapp/web && npx tsc --noEmit`
Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add webapp/web/src/
git commit -m "feat(c2-ui): add MissionPicker component + REST API client"
```

---

## Task 17: AppShell — wire MissionPicker + CesiumViewer + frame stream

**Files:**
- Modify: `webapp/web/src/AppShell.tsx`
- Create: `webapp/web/src/components/EntityList.tsx`
- Create: `webapp/web/src/components/panels/Placeholder.tsx`

- [ ] **Step 1: Create `webapp/web/src/components/panels/Placeholder.tsx`** (stub panels for future features)

```tsx
export interface PlaceholderProps { title: string; description: string; }

export function Placeholder({ title, description }: PlaceholderProps) {
  return (
    <div style={{ padding: 12, borderBottom: '1px solid #333', color: '#aaa' }}>
      <div style={{ fontWeight: 'bold', color: '#ccc', marginBottom: 4 }}>{title}</div>
      <div style={{ fontSize: 12 }}>{description}</div>
    </div>
  );
}
```

- [ ] **Step 2: Create `webapp/web/src/components/EntityList.tsx`**

```tsx
import type { FrameDto } from '../types';

export interface EntityListProps { frame: FrameDto | null; }

export function EntityList({ frame }: EntityListProps) {
  if (!frame) return <div style={{ padding: 12, color: '#666' }}>No frames yet</div>;
  const sorted = [...frame.entities].sort((a, b) => a.teamId - b.teamId || a.id - b.id);
  return (
    <div style={{ padding: 8, fontSize: 13, color: '#ccc' }}>
      <div style={{ marginBottom: 6, color: '#888' }}>
        Entities: {frame.entities.length} (t={frame.time.toFixed(1)})
      </div>
      {sorted.map(e => (
        <div key={e.id} style={{
          padding: '3px 6px',
          color: e.teamId === 1 ? '#7af' : e.teamId === 2 ? '#f77' : '#ccc',
          opacity: e.active ? 1 : 0.4,
        }}>
          #{e.id} · team {e.teamId} · {e.type}
        </div>
      ))}
    </div>
  );
}
```

- [ ] **Step 3: Replace `webapp/web/src/AppShell.tsx`** with the full layout

```tsx
import { useRef, useState } from 'react';
import { CesiumViewer, type ViewerHandle } from './components/CesiumViewer';
import { MissionPicker } from './components/MissionPicker';
import { EntityList } from './components/EntityList';
import { Placeholder } from './components/panels/Placeholder';
import { useFrameStream } from './hooks/useFrameStream';
import type { MissionStartResponse } from './types';

export function AppShell() {
  const viewerRef = useRef<ViewerHandle | null>(null);
  const [streamConnected, setStreamConnectedDisplay] = useState(false);

  const { connected, latestFrame } = useFrameStream(frame => {
    viewerRef.current?.applyFrame(frame);
  });
  if (connected !== streamConnected) setStreamConnectedDisplay(connected);

  function handleStarted(resp: MissionStartResponse) {
    viewerRef.current?.setOrigin(resp.origin);
  }
  function handleStopped() {
    // Leave the last frame on screen until the next start; nothing to do here.
  }

  return (
    <div style={{ display: 'grid', gridTemplateRows: 'auto 1fr', height: '100vh', background: '#111' }}>
      {/* Header */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
        <MissionPicker onStarted={handleStarted} onStopped={handleStopped} />
        <span style={{ color: connected ? '#7f7' : '#f77', marginRight: 16 }}>
          ● Stream {connected ? 'connected' : 'disconnected'}
        </span>
      </div>

      {/* Three-column body: sidebar | viewer | right rail */}
      <div style={{ display: 'grid', gridTemplateColumns: '240px 1fr 280px', minHeight: 0 }}>
        <aside style={{ background: '#1a1a1a', overflowY: 'auto', borderRight: '1px solid #333' }}>
          <div style={{ padding: 8, color: '#888', fontSize: 11, textTransform: 'uppercase' }}>Entities</div>
          <EntityList frame={latestFrame} />
        </aside>

        <main style={{ position: 'relative' }}>
          <CesiumViewer onReady={h => { viewerRef.current = h; }} />
        </main>

        <aside style={{ background: '#1a1a1a', overflowY: 'auto', borderLeft: '1px solid #333' }}>
          <Placeholder title="Commands" description="Operator commands (target_assignment, swap_team) — coming in v2." />
          <Placeholder title="Topic Stream" description="Subscribe to a drone's pub/sub topics — needs a SCRIMMAGE TopicTap plugin (v2)." />
          <Placeholder title="Tags" description="Annotate entities with operator-defined labels — v2." />
        </aside>
      </div>
    </div>
  );
}
```

- [ ] **Step 4: Build to verify compile**

Run: `cd webapp/web && npx tsc --noEmit`
Expected: no errors.

- [ ] **Step 5: Commit**

```bash
git add webapp/web/src/
git commit -m "feat(c2-ui): wire AppShell with MissionPicker, viewer, entity list, placeholder panels"
```

---

## Task 18: Full-stack end-to-end smoke test (the demo flow)

This is the spec's "Manual smoke test" from §8 — the only test that absolutely must pass to call MVP done.

**Files:** none (verification only)

- [ ] **Step 1: Set the Cesium token**

Confirm `webapp/.env` exists with `CESIUM_ION_TOKEN=<your real token>`.

- [ ] **Step 2: Bring up the full stack**

Run: `cd webapp && docker compose up --build`
Expected: all four containers start. Watch for:
- `c2-postgres` ready
- `c2-scrimmage` `* Running on http://0.0.0.0:5050`
- `c2-api` `Now listening on: http://[::]:8080`
- `c2-web` `Local:   http://localhost:5173/`

- [ ] **Step 3: Open the browser**

Navigate to http://localhost:5173
Expected: header shows mission dropdown (defaulted to `capture-the-flag.xml`), Start/Stop buttons, "Stream connected" indicator green within ~2s. Cesium globe visible in the center pane.

- [ ] **Step 4: Start the mission**

Click **Start**. Within ~3s:
- Camera flies to Camp Roberts.
- ~20 colored points appear (blue = team 1, red = team 2) over the terrain.
- Points move as the mission advances.
- Sidebar shows entity list with IDs and team colors, updating ~5x/sec.
- "Running: capture-the-flag.xml" appears in the header.

- [ ] **Step 5: Verify entity disappearance on collision**

Watch for ~30s. As drones collide, points disappear from the map and from the sidebar list. (The capture-the-flag mission has aggressive boundary defense; collisions are expected.)

- [ ] **Step 6: Stop and switch missions**

Click **Stop** → "Idle" badge.
Pick another mission with a geographic origin (e.g., `predator_prey_boids.xml`, `straight.xml` if it has origins set — try a few).
Click **Start** → new mission renders. Origin update flies the camera to the new location if different.

- [ ] **Step 7: Test mid-mission browser refresh**

While a mission is running, hit Ctrl+R in the browser. Expected: page reloads, viewer reinitializes, frames resume within 2-3s. (The SCRIMMAGE process keeps running across the page reload because it's container-side.)

- [ ] **Step 8: Test error path — pick a no-origin mission**

If you have one (e.g., a 2D mission without geographic origins set), try starting it. Expected: error toast/banner with "no geographic origin" message; previous mission state remains.

- [ ] **Step 9: Tear down**

Ctrl+C the compose process; `docker compose down`.

- [ ] **Step 10: Commit any fixes from the smoke test**

```bash
git add -A
git commit -m "fix(c2-ui): smoke-test fixes" --allow-empty
```

> **If the smoke test fails:**
> - **Frames don't reach the browser but API logs show "Frame stream opened":** SignalR or CORS issue. Check browser dev tools network tab for the `/hubs/frames` connection (should upgrade to WebSocket). CORS errors → the `WithOrigins("http://localhost:5173")` in `Program.cs` is the most likely culprit.
> - **API logs never show "Frame stream opened":** scrimmage isn't reaching the API. Check that the templated `/tmp/active_mission.xml` inside the scrimmage container has `<stream_ip>api</stream_ip>` and `<stream_port>8080</stream_port>`. Also verify `network_gui="true"` in the templated file.
> - **Drones in the wrong place / in the ocean:** `enuToCartesian` is wrong — re-run `npm test` and add a debug log of the first frame's position + computed Cartesian.
> - **Cesium shows blank globe / no terrain:** Ion token missing or invalid. Check the browser console.

---

## Task 19: Polish — error toasts + status badge wiring

The smoke test in Task 18 covers the happy path. This task tightens the user-visible error story.

**Files:**
- Modify: `webapp/web/src/components/MissionPicker.tsx` (already shows error inline — verify it covers all error paths)
- Modify: `webapp/web/src/components/CesiumViewer.tsx` (add Ion-failure banner)

- [ ] **Step 1: Add Cesium Ion-failure banner to `CesiumViewer.tsx`**

In the `useEffect`, replace the `Cesium.createWorldTerrainAsync()` block with one that surfaces failures via state:

```tsx
// Add at top of component, alongside other refs:
const [terrainError, setTerrainError] = useState<string | null>(null);

// Inside useEffect, replace the createWorldTerrainAsync block with:
if (ION_TOKEN) {
  Cesium.createWorldTerrainAsync()
    .then(t => { viewer.terrainProvider = t; })
    .catch(err => {
      console.warn('Cesium terrain load failed:', err);
      setTerrainError('3D terrain unavailable — check Cesium Ion token');
    });
} else {
  setTerrainError('3D terrain unavailable — set VITE_CESIUM_ION_TOKEN');
}

// Below the <div ref={ref}> in the return, wrap to show the banner:
return (
  <>
    <div ref={ref} style={{ width: '100%', height: '100%' }} />
    {terrainError && (
      <div style={{
        position: 'absolute', top: 8, left: '50%', transform: 'translateX(-50%)',
        background: 'rgba(255,200,0,0.9)', color: '#000', padding: '6px 12px',
        borderRadius: 4, fontSize: 12,
      }}>{terrainError}</div>
    )}
  </>
);
```

> If the JSX root needs to wrap in a fragment, add `<>` and `</>`.

- [ ] **Step 2: Verify the error path manually**

Temporarily blank `VITE_CESIUM_ION_TOKEN` in `webapp/.env`. Restart `docker compose up web`. Reload the page. Expected: globe still renders (Cesium has a default imagery provider) and the yellow banner appears at top.

Restore your real token before continuing.

- [ ] **Step 3: Commit**

```bash
git add webapp/web/src/components/CesiumViewer.tsx
git commit -m "feat(c2-ui): show banner when Cesium Ion token is missing or invalid"
```

---

## Task 20: README finalize

**Files:**
- Modify: `webapp/README.md`

- [ ] **Step 1: Replace `webapp/README.md` with the final version**

```markdown
# SCRIMMAGE C2 UI (MVP)

Web-based command-and-control UI for SCRIMMAGE missions. Built on a forked SCRIMMAGE.

## What it does (MVP)

- 3D live visualization of any pre-existing SCRIMMAGE mission with a geographic origin
- Mission selector dropdown with Start / Stop controls
- Operator console layout (sidebar entity list, Cesium viewer, placeholder panels for v2 features)

## Architecture (MVP)

Four containers in one `docker-compose.yml`:

- **`scrimmage`** — GTRI base image + Flask launcher sidecar. Listens on `:5050` for mission lifecycle, exposes scrimmage's gRPC frame stream port.
- **`api`** — .NET 10 minimal API. Hosts the `ScrimmageService` gRPC server (scrimmage pushes `Frame` protos to it), broadcasts via SignalR, proxies REST mission lifecycle to the launcher.
- **`web`** — React + Vite + Cesium. Subscribes to the SignalR frame stream, renders entities on a 3D globe with Ion terrain.
- **`postgres`** — scaffolded with empty schema; **unused in MVP**, ready for v2 (commands / audit / tags).

See `docs/specs/2026-05-02-scrimmage-c2-ui-design.md` for the full design.
See `docs/plans/2026-05-02-scrimmage-c2-ui-mvp.md` for the implementation plan that built this.

## Quick start

1. Get a free Cesium Ion access token at https://cesium.com/ion/
2. `cp webapp/.env.example webapp/.env` and paste your token into `CESIUM_ION_TOKEN`.
3. `cd webapp && docker compose up --build`
4. Open http://localhost:5173, pick `capture-the-flag.xml`, click **Start**.

## Demo missions

Any SCRIMMAGE mission XML in `missions/` that has `<latitude_origin>`, `<longitude_origin>`, and `<altitude_origin>` set will work. `capture-the-flag.xml` is the recommended demo target — anchored at Camp Roberts, CA, with two visible teams of UAVs.

Missions without geographic origins return a 400 from the launcher and surface as an error toast in the UI. This is intentional — the Cesium viewer requires geographic anchoring.

## Known limitations

- Static mission geometry (boundaries, flags) is not rendered. They're SCRIMMAGE `entity_interaction` plugins, not entities, and don't appear in the frame stream.
- Operator commands (target assignment, team swap) are not implemented in MVP — see `docs/specs/…` §10.
- Live pub/sub topic visualization, entity tagging, audit logging, pause/resume, and frame scrubbing are all v2.
```

- [ ] **Step 2: Commit**

```bash
git add webapp/README.md
git commit -m "docs(c2-ui): finalize webapp README"
```

---

## Plan complete

If everything above passes, the MVP is done:
- Four containers come up cleanly via `docker compose up --build`
- Operator picks a mission from the dropdown, clicks Start, and sees drones flying in 3D over real terrain
- Stop, pick a different mission, Start — works
- Browser refresh mid-mission — reconnects cleanly
- The whole stack lives in `webapp/` inside the SCRIMMAGE fork, ready for v2 features to slot into the placeholder panels

Future work (v2) is documented in the spec at `webapp/docs/specs/2026-05-02-scrimmage-c2-ui-design.md` §10.
