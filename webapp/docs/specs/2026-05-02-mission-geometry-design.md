# Static Mission Geometry — Design Spec

**Date:** 2026-05-02
**Author:** Scott McCutchen (with Claude)
**Status:** Approved for implementation (amended 2026-05-02)
**Scope:** Hackathon v1 — render mission boundaries, flags, and capture-zone shells in the Cesium viewer

---

## Amendment (2026-05-02, post-approval)

The original spec assumed a C# parser. During plan-writing it surfaced that the mission XML is templated by a **Python/Flask launcher sidecar** (`webapp/scrimmage-runner/launcher/app.py`), not by the C# API, and the templated XML lives in the scrimmage container's filesystem (`/tmp/active_mission.xml`). The C# API has no path to it.

**Amended approach:**

- Parser lives in the Python launcher, runs immediately after `_template_mission` produces the templated XML string.
- `/missions/start` response gains a `geometry` field carrying the parsed DTO.
- C# `/api/missions/start` proxy already forwards the launcher response unchanged → no C# code change required.
- Frontend reads `geometry` directly from the start response (no SignalR event for v1).

**Implications:**

- `webapp/api/MissionGeometry.cs` — not created.
- `Dtos.cs` — extended only for TS-side type mirroring (no actual API serialization needed; the field rides through as JSON).
- `FrameHub` — unchanged. No `GetCurrentGeometry`, no `OnSceneSetup`, no `OnSceneClear`.
- Late-joining browsers don't see the geometry until next Start. Acceptable for hackathon (single-operator, single-tab demo).
- All §4 sections referencing C# parsing or SignalR scene events are superseded by the plan; the rest of the spec (parser semantics, coordinate handling, render approach, error handling) still applies.

---

## 1. Problem

SCRIMMAGE missions define static world geometry (team boundaries, flags, capture zones) as `<entity_interaction>` plugins in the mission XML. These are not entities and never appear in the gRPC frame stream, so the C2 UI's Cesium viewer renders the drones flying through invisible geometry. For `capture-the-flag.xml` this means the operator cannot see the playing field, the flag positions, or the capture zones — only the drones moving in space.

This spec covers a server-side parse of the mission XML and a one-shot push to the client to render those shapes as static Cesium primitives for the duration of the mission.

This is implementation path #1 from the original design (`webapp/docs/specs/2026-05-02-scrimmage-c2-ui-design.md` §10). Path #2 (wire up SCRIMMAGE's `SendShapes` gRPC) is deferred — it would let plugins push shapes at runtime but requires C++ changes.

## 2. Scope

**In scope (v1):**

- Parse `<entity_interaction type="cuboid">` and `<entity_interaction type="sphere">` tags from the templated mission XML.
- Parse `CaptureInBoundaryInteraction` tags and render a translucent shell around the referenced boundary (option B from brainstorm).
- Push parsed geometry to the client once, at mission start, via SignalR.
- Render in Cesium as box / ellipsoid primitives using color and opacity from the XML.
- Clear the geometry layer on mission stop or new mission start.

**Out of scope (v1):**

- Any `entity_interaction` shape with a `type` attribute we don't recognize (skip + log).
- `FlagCaptureInteraction` linkage lines (option C from brainstorm).
- Per-shape visibility toggles in the UI.
- Mid-mission geometry updates (use path #2 for that).
- Visual model meshes for shapes (XML doesn't carry them anyway).

## 3. Architecture

One C# parser, one new SignalR event, one new Cesium render pass on the client. No proto changes, no SCRIMMAGE-side changes, no DB.

```
Mission XML  ──→  Launcher templates XML  ──→  MissionGeometry.Parse(templatedXml)
                                                        │
                                                        ▼
                                              MissionGeometryDto
                                                        │
                                       FrameHub.Clients.All.SendAsync("OnSceneSetup", dto)
                                                        │
                                                        ▼
                                          useFrameStream → geometry state
                                                        │
                                                        ▼
                                       MissionGeometryLayer (Cesium primitives)
```

## 4. Components

### 4.1 API: `webapp/api/MissionGeometry.cs` (new)

Pure parser. No I/O, no SignalR coupling.

**Public surface:**

```csharp
public static class MissionGeometry
{
    public static MissionGeometryDto Parse(string templatedMissionXml);
}
```

**Behavior:**

- Loads the XML, walks all `<entity_interaction>` children of the root.
- For each tag with `type="cuboid"` or `type="sphere"` and explicit `center`, build a `GeometryShape`.
- For each tag whose body text is `CaptureInBoundaryInteraction`, build a `CaptureZone` referencing the shape by its `boundary_id`.
- Skip and log any tag whose `type` is missing or unrecognized.
- Tolerant of missing optional attributes (default `team_id=0`, `opacity=1.0`, `color="255 255 255"`).

### 4.2 API: `webapp/api/Dtos.cs` (extend)

```csharp
public record MissionGeometryDto(
    List<GeometryShape> Shapes,
    List<CaptureZone> CaptureZones);

public record GeometryShape(
    int Id,
    string Name,
    int TeamId,
    string Kind,            // "cuboid" | "sphere"
    double[] Center,        // length 3, ENU meters
    double[]? Lengths,      // length 3, only for cuboid
    double? Radius,         // only for sphere
    int[] Color,            // length 3, 0-255 RGB
    double Opacity);

public record CaptureZone(
    string Name,
    int BoundaryId,
    double CaptureRange);
```

### 4.3 API: launcher / mission-start path (extend)

Wherever the launcher templates the mission XML on Start (per project memory: forces `network_gui="true"`, injects `<stream_ip>`, etc.), after writing the templated XML:

1. Call `MissionGeometry.Parse(templatedXml)`.
2. Stash the resulting DTO in a per-session field on the launcher service (so late SignalR joiners can be sent it on connect — see §4.5).
3. Broadcast on `FrameHub` immediately after the SCRIMMAGE process is launched (no need to wait for the first frame; the client renders geometry independently): `await _frameHub.Clients.All.SendAsync("OnSceneSetup", dto);`
4. On mission Stop or new Start, broadcast `OnSceneClear` (no payload) so the client wipes the layer. The frontend handler sets the `geometry` state to `null`, which triggers the layer component's cleanup path.

The parser must read the **templated** XML (post-launcher modifications), not the original on disk, because the templating step is the canonical pre-run snapshot.

### 4.4 API: `webapp/api/FrameHub.cs` (extend)

Currently a stub. Add a hub method clients can call on connect to fetch the current geometry, so a late-joining browser doesn't miss the one-shot:

```csharp
public class FrameHub : Hub
{
    private readonly LauncherService _launcher;  // or wherever current DTO lives
    public FrameHub(LauncherService launcher) { _launcher = launcher; }

    public MissionGeometryDto? GetCurrentGeometry()
        => _launcher.CurrentGeometry;
}
```

### 4.5 Web: `webapp/web/src/hooks/useFrameStream.ts` (extend)

- Subscribe to `OnSceneSetup` and `OnSceneClear` alongside `OnFrame`.
- Expose `geometry: MissionGeometryDto | null` in the hook return.
- On `connect` (initial start AND reconnects), invoke `conn.invoke("GetCurrentGeometry")` and set the state if non-null. Handles late-join and reconnect cleanly.

### 4.6 Web: `webapp/web/src/types.ts` (extend)

Mirror the C# DTOs in TypeScript: `MissionGeometryDto`, `GeometryShape`, `CaptureZone`.

### 4.7 Web: `webapp/web/src/components/MissionGeometryLayer.tsx` (new)

A Cesium component that takes `geometry` and the mission's lat/lon/alt origin (already known to the viewer) and renders one Entity per shape and one per capture zone.

**Coordinate conversion:**

SCRIMMAGE local frame is ENU (East-North-Up) meters relative to the mission origin. Use `Cesium.Transforms.eastNorthUpToFixedFrame(originCartesian)` once, then `Cesium.Matrix4.multiplyByPoint(matrix, localOffset, result)` per shape to get a world-frame Cartesian position.

**Rendering:**

- `cuboid` → `Entity` with `box: { dimensions: new Cartesian3(lx, ly, lz), material: Color.fromBytes(r, g, b, a*255) }`.
- `sphere` → `Entity` with `ellipsoid: { radii: new Cartesian3(r, r, r), material: ... }`.
- Capture zone → look up the referenced boundary by `boundaryId`, render an additional translucent box at `lengths + 2*captureRange` with a contrasting fill (e.g., shape color at 0.15 opacity + outline). If the referenced boundary isn't a cuboid, skip with a console warn.

The component owns its Cesium entities and removes them on unmount or when `geometry` becomes null.

## 5. Data flow

1. Operator clicks **Start** → API templates mission XML, parses geometry, starts SCRIMMAGE process.
2. API broadcasts `OnSceneSetup` with the DTO.
3. Frontend hook receives event, sets state, `MissionGeometryLayer` mounts the Cesium entities.
4. Frame stream begins; drones render on top of the static geometry.
5. Late-joining browser tab connects → calls `GetCurrentGeometry` → receives same DTO → renders.
6. Operator clicks **Stop** (or mission ends naturally) → API broadcasts `OnSceneClear` → frontend wipes layer.
7. Operator clicks **Start** with a different mission → API broadcasts `OnSceneClear` then `OnSceneSetup` with new DTO.

## 6. Error handling

- **Malformed XML:** parser throws; launcher logs and proceeds without geometry (mission still runs, just no static visuals). Don't fail the start.
- **Unknown shape type:** skip with structured log entry. Don't throw.
- **Missing required attribute on a recognized shape** (e.g., `cuboid` without `lengths`): skip that shape with a log entry.
- **Capture zone references unknown boundary_id:** skip with log entry.
- **Client receives geometry before viewer's mission origin is known:** the layer waits for both before mounting entities.

## 7. Testing

Manual verification only (hackathon scope):

- Launch `capture-the-flag.xml`; confirm 2 boxes (blue/red) and 2 spheres (blue/red flags) appear, plus 2 capture-zone shells. Eyeball positions match the field layout.
- Launch `predator_prey_boids.xml` (no `entity_interaction` shapes); confirm no geometry renders, no errors.
- Stop and re-Start with a different mission; confirm clean swap, no leftover entities.
- Reload the browser mid-mission; confirm geometry re-appears via `GetCurrentGeometry`.
- Coordinate-sign sanity: blue boundary should appear east of the origin, red west. If flipped, the local frame is NED not ENU; swap axes in the layer component.

No automated tests for v1.

## 8. Risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| Local frame is NED, not ENU | Low (~10%) | 5-min fix in `MissionGeometryLayer`: swap axis components |
| Templated-XML file path not exposed by launcher | Medium | Parser accepts a string, not a path; pass whatever the launcher has in memory |
| `FrameHub` DI doesn't have `LauncherService` (or equivalent) | Low | Use a singleton `MissionGeometryStore` injected into both hub and launcher |
| Cesium box `dimensions` interpreted in non-ENU frame | Low | Use `orientation` from the same `eastNorthUpToFixedFrame` matrix on each entity |

## 9. Estimated effort

~1.5 hours of focused work if risks hold:

- Parser + DTO: 30 min
- Launcher wire-up + SignalR plumbing: 20 min
- Cesium render component: 30 min
- Integration + manual verify on capture-the-flag: 15 min
- Padding for coordinate-sign flip and small Cesium quirks: 15 min

## 10. Follow-ups (not v1)

- Option C polish: linkage lines from `FlagCaptureInteraction` (flag → capture zone).
- Per-shape visibility toggles in the right-rail panel.
- Migrate to path #2 (`SendShapes` gRPC) when we want runtime-dynamic shapes from autonomy plugins.
