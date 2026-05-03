# Static Mission Geometry v1 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render mission boundaries, flags, and capture-zone shells in the Cesium viewer by parsing the templated mission XML server-side and shipping a geometry payload to the frontend on mission Start.

**Architecture:** Python launcher parses `<entity_interaction type="cuboid|sphere">` and `CaptureInBoundaryInteraction` tags from the templated XML it just produced, returns the geometry as a new field in the `/missions/start` JSON response. The C# `/api/missions/start` proxy already forwards launcher responses unchanged. Frontend reads `geometry` from the start response, hands it to the Cesium viewer, viewer renders box/ellipsoid primitives until cleared on mission Stop.

**Tech Stack:** Python 3 + `xml.etree.ElementTree` (parser, already imported). React + TypeScript + Cesium (renderer). No new dependencies.

**Spec:** `webapp/docs/specs/2026-05-02-mission-geometry-design.md` (with 2026-05-02 amendment for Python-side parsing).

**Time budget:** ~1.5 hours.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `webapp/scrimmage-runner/launcher/app.py` | Modify | Add `_parse_geometry(text)` pure function. Call it after `_template_mission`, include result in `/missions/start` response under `geometry` key. |
| `webapp/web/src/types.ts` | Modify | Add `GeometryShape`, `CaptureZone`, `MissionGeometry` types. Extend `MissionStartResponse` with optional `geometry` field. |
| `webapp/web/src/components/CesiumViewer.tsx` | Modify | Add `setGeometry(geo)` to `ViewerHandle`. Manage a `Map<string, Cesium.Entity>` for geometry entities. Clear on `null`, render on set. |
| `webapp/web/src/AppShell.tsx` | Modify | In `handleStarted`, call `viewerRef.current?.setGeometry(resp.geometry ?? null)`. In `handleStopped`, call `setGeometry(null)`. |

No new files. Each modification is bounded and independently testable.

---

## Task 1: Python parser (smoke-tested in isolation)

**Files:**
- Modify: `webapp/scrimmage-runner/launcher/app.py` (add `_parse_geometry` function only — wire-up is Task 2)

- [ ] **Step 1: Add the `_parse_geometry` function**

Insert immediately after the `_template_mission` function (around line 120 of `app.py`):

```python
def _parse_geometry(text: str) -> dict:
    """Parse <entity_interaction> shape tags from a templated mission XML string.

    Returns a dict shaped {shapes: [...], captureZones: [...]} suitable for JSON
    serialization. Only tags with type="cuboid" or type="sphere" and an explicit
    center are emitted as shapes. CaptureInBoundaryInteraction tags become
    captureZones referencing a shape by its boundary_id.

    Tolerant of malformed input: returns empty lists if XML is unparseable.
    Skips unrecognized shape types and shapes missing required attributes
    (cuboid needs lengths, sphere needs radius), logging to stdout.
    """
    shapes = []
    capture_zones = []
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        print(f"[geometry] XML parse failed: {e}", flush=True)
        return {"shapes": shapes, "captureZones": capture_zones}

    for tag in root.findall("entity_interaction"):
        body = (tag.text or "").strip()
        # Renderable shape tag (body is "Boundary" in capture-the-flag)
        shape_type = tag.get("type")
        center_attr = tag.get("center")
        if shape_type in ("cuboid", "sphere") and center_attr:
            try:
                center = [float(x.strip()) for x in center_attr.split(",")]
                if len(center) != 3:
                    raise ValueError(f"center must have 3 components, got {len(center)}")

                color_attr = tag.get("color", "255 255 255").split()
                color = [int(c) for c in color_attr[:3]] if len(color_attr) >= 3 else [255, 255, 255]

                shape = {
                    "id": int(tag.get("id", 0)),
                    "name": tag.get("name", ""),
                    "teamId": int(tag.get("team_id", 0)),
                    "kind": shape_type,
                    "center": center,
                    "color": color,
                    "opacity": float(tag.get("opacity", 1.0)),
                }
                if shape_type == "cuboid":
                    lengths_attr = tag.get("lengths")
                    if not lengths_attr:
                        print(f"[geometry] skipping cuboid '{shape['name']}': no lengths", flush=True)
                        continue
                    lengths = [float(x.strip()) for x in lengths_attr.split(",")]
                    if len(lengths) != 3:
                        print(f"[geometry] skipping cuboid '{shape['name']}': lengths must have 3 components", flush=True)
                        continue
                    shape["lengths"] = lengths
                else:  # sphere
                    radius_attr = tag.get("radius")
                    if not radius_attr:
                        print(f"[geometry] skipping sphere '{shape['name']}': no radius", flush=True)
                        continue
                    shape["radius"] = float(radius_attr)
                shapes.append(shape)
            except (ValueError, TypeError) as e:
                print(f"[geometry] skipping shape '{tag.get('name', '?')}': {e}", flush=True)
            continue

        # Capture zone (body is "CaptureInBoundaryInteraction")
        if body == "CaptureInBoundaryInteraction":
            try:
                capture_zones.append({
                    "name": tag.get("name", ""),
                    "boundaryId": int(tag.get("boundary_id", 0)),
                    "captureRange": float(tag.get("capture_range", 0)),
                })
            except (ValueError, TypeError) as e:
                print(f"[geometry] skipping capture zone '{tag.get('name', '?')}': {e}", flush=True)

    return {"shapes": shapes, "captureZones": capture_zones}
```

- [ ] **Step 2: Smoke-test the parser against capture-the-flag**

The launcher container has python and the missions dir mounted. Run a one-off command in the running scrimmage container (or equivalent):

```bash
docker compose exec scrimmage python -c "
import sys
sys.path.insert(0, '/app/launcher')
from app import _parse_geometry
text = open('/root/scrimmage/scrimmage/missions/capture-the-flag.xml').read()
import json
print(json.dumps(_parse_geometry(text), indent=2))
"
```

Expected output: 4 shapes (`blue_boundary` cuboid id=1, `blue_flag` sphere id=2, `red_boundary` cuboid id=3, `red_flag` sphere id=4), 2 captureZones (`BlueCaptureBoundary` boundaryId=1, `RedCaptureBoundary` boundaryId=3). No `[geometry] skipping ...` log lines.

If the container path differs, adapt the path; the test is "give the function the file content, eyeball the JSON".

- [ ] **Step 3: Smoke-test against predator_prey_boids (no shapes case)**

```bash
docker compose exec scrimmage python -c "
import sys; sys.path.insert(0, '/app/launcher')
from app import _parse_geometry
text = open('/root/scrimmage/scrimmage/missions/predator_prey_boids.xml').read()
import json; print(json.dumps(_parse_geometry(text), indent=2))
"
```

Expected: `{"shapes": [], "captureZones": []}`. No errors.

- [ ] **Step 4: Commit**

```bash
git add webapp/scrimmage-runner/launcher/app.py
git commit -m "feat(launcher): parse static mission geometry from templated XML"
```

---

## Task 2: Wire parser into `/missions/start` response

**Files:**
- Modify: `webapp/scrimmage-runner/launcher/app.py` (start_mission handler, ~line 130-181)

- [ ] **Step 1: Call parser after templating, include in response**

Inside `start_mission()`, after the `try: origin = _template_mission(...)` block (around line 153), and before the `proc = subprocess.Popen(...)` call, add:

```python
    # Parse static geometry (boundaries, flags, capture zones) from the *templated* file.
    # Templated XML lives at ACTIVE_MISSION_PATH; safe to re-read since we just wrote it.
    try:
        geometry = _parse_geometry(ACTIVE_MISSION_PATH.read_text())
    except Exception as e:
        print(f"[geometry] parse failed (non-fatal): {e}", flush=True)
        geometry = {"shapes": [], "captureZones": []}
```

Then update the final `return jsonify({...})` block at the bottom of `start_mission` to include `"geometry": geometry`:

```python
    return jsonify({
        "status": "started",
        "pid": proc.pid,
        "mission": name,
        "origin": origin,
        "timeWarp": time_warp,
        "geometry": geometry,
    })
```

- [ ] **Step 2: Restart the launcher container and verify the response**

```bash
docker compose restart scrimmage
curl -X POST http://localhost:5050/missions/start \
  -H "Content-Type: application/json" \
  -d '{"name": "capture-the-flag.xml", "timeWarp": 5}'
```

Expected: response includes `"geometry": {"shapes": [...4 items...], "captureZones": [...2 items...]}` alongside the existing `origin`/`pid`/`mission` fields.

Stop the test mission:

```bash
curl -X POST http://localhost:5050/missions/stop
```

- [ ] **Step 3: Verify pass-through to C# proxy**

The C# proxy at `webapp/api/Endpoints.cs:68-76` forwards the launcher body verbatim, so no C# change is needed. Verify:

```bash
curl -X POST http://localhost:8080/api/missions/start \
  -H "Content-Type: application/json" \
  -d '{"name": "capture-the-flag.xml", "timeWarp": 5}'
```

Expected: same response shape, `geometry` field present. Stop again with `curl -X POST http://localhost:8080/api/missions/stop`.

- [ ] **Step 4: Commit**

```bash
git add webapp/scrimmage-runner/launcher/app.py
git commit -m "feat(launcher): include geometry payload in /missions/start response"
```

---

## Task 3: Add TypeScript types

**Files:**
- Modify: `webapp/web/src/types.ts` (append types, extend `MissionStartResponse`)

- [ ] **Step 1: Append geometry types and extend `MissionStartResponse`**

At the end of `webapp/web/src/types.ts`, add:

```typescript
export interface GeometryShape {
  id: number;
  name: string;
  teamId: number;
  kind: 'cuboid' | 'sphere';
  /** ENU offset from mission origin, in meters: [east, north, up] */
  center: [number, number, number];
  /** Cuboid only: [lx, ly, lz] in meters along ENU axes. */
  lengths?: [number, number, number];
  /** Sphere only: radius in meters. */
  radius?: number;
  /** RGB 0-255 */
  color: [number, number, number];
  /** 0.0 - 1.0 */
  opacity: number;
}

export interface CaptureZone {
  name: string;
  boundaryId: number;
  captureRange: number;
}

export interface MissionGeometry {
  shapes: GeometryShape[];
  captureZones: CaptureZone[];
}
```

Then modify the existing `MissionStartResponse` interface (around line 22) to add an optional `geometry` field:

```typescript
export interface MissionStartResponse {
  status: string;
  pid: number;
  mission: string;
  origin: Origin;
  timeWarp?: number | null;
  geometry?: MissionGeometry;
}
```

- [ ] **Step 2: Verify TypeScript compiles**

```bash
docker compose exec web npm run build
# OR if dev server is hot-reloading:
docker compose logs --tail 50 web
```

Expected: no type errors. The new optional field doesn't break existing call sites.

- [ ] **Step 3: Commit**

```bash
git add webapp/web/src/types.ts
git commit -m "feat(web): add MissionGeometry types and extend MissionStartResponse"
```

---

## Task 4: Cesium geometry rendering

**Files:**
- Modify: `webapp/web/src/components/CesiumViewer.tsx` (add `setGeometry` to handle, manage geometry entities)

- [ ] **Step 1: Extend `ViewerHandle` interface**

In `webapp/web/src/components/CesiumViewer.tsx`, add `MissionGeometry` to the type imports at line 3:

```typescript
import type { FrameDto, Origin, MissionGeometry } from '../types';
```

Add a new method to the `ViewerHandle` interface (around line 15-37). Insert after `setOrigin`:

```typescript
  /**
   * Replace the static mission geometry (boundaries, flags, capture zones).
   * Pass null to clear. Geometry is rendered relative to the current origin —
   * call setOrigin first if both change in the same operation.
   */
  setGeometry: (geometry: MissionGeometry | null) => void;
```

- [ ] **Step 2: Add a geometry-entity ref and renderer helper**

After `targetLineEntityRef` (around line 58), add:

```typescript
  const geometryEntitiesRef = useRef<Cesium.Entity[]>([]);
```

At the bottom of the file (after the `applyFrame` function), add a new helper:

```typescript
function renderGeometry(
  geometry: MissionGeometry | null,
  viewer: Cesium.Viewer,
  geometryEntities: Cesium.Entity[],
  origin: Origin | null,
) {
  // Always clear existing geometry entities first — both for null (explicit clear)
  // and for replacement (mission swap).
  for (const ent of geometryEntities) {
    viewer.entities.remove(ent);
  }
  geometryEntities.length = 0;

  if (!geometry || !origin) return;

  // Build a quick lookup so capture zones can find their referenced shape by id.
  const shapeById = new Map<number, GeometryShape>();
  for (const s of geometry.shapes) shapeById.set(s.id, s);

  // Render shapes
  for (const shape of geometry.shapes) {
    const center = enuToCartesian(origin, shape.center[0], shape.center[1], shape.center[2]);
    const orientation = Cesium.Transforms.headingPitchRollQuaternion(
      center,
      new Cesium.HeadingPitchRoll(0, 0, 0),
    );
    const fillColor = Cesium.Color.fromBytes(
      shape.color[0], shape.color[1], shape.color[2],
      Math.round(shape.opacity * 255),
    );
    const outlineColor = Cesium.Color.fromBytes(
      shape.color[0], shape.color[1], shape.color[2], 255,
    );
    if (shape.kind === 'cuboid' && shape.lengths) {
      const ent = viewer.entities.add({
        position: center,
        orientation,
        box: {
          dimensions: new Cesium.Cartesian3(
            shape.lengths[0], shape.lengths[1], shape.lengths[2],
          ),
          material: fillColor,
          outline: true,
          outlineColor,
          outlineWidth: 2,
        },
      });
      geometryEntities.push(ent);
    } else if (shape.kind === 'sphere' && shape.radius != null) {
      const ent = viewer.entities.add({
        position: center,
        ellipsoid: {
          radii: new Cesium.Cartesian3(shape.radius, shape.radius, shape.radius),
          material: fillColor,
          outline: true,
          outlineColor,
        },
      });
      geometryEntities.push(ent);
    }
  }

  // Render capture-zone shells around their referenced cuboid boundaries.
  // Skip if the referenced shape isn't a cuboid (no clear shell shape for spheres).
  for (const zone of geometry.captureZones) {
    const ref = shapeById.get(zone.boundaryId);
    if (!ref || ref.kind !== 'cuboid' || !ref.lengths) {
      console.warn(`[geometry] capture zone ${zone.name} references non-cuboid boundary ${zone.boundaryId}; skipping shell`);
      continue;
    }
    const center = enuToCartesian(origin, ref.center[0], ref.center[1], ref.center[2]);
    const orientation = Cesium.Transforms.headingPitchRollQuaternion(
      center,
      new Cesium.HeadingPitchRoll(0, 0, 0),
    );
    const expanded = new Cesium.Cartesian3(
      ref.lengths[0] + 2 * zone.captureRange,
      ref.lengths[1] + 2 * zone.captureRange,
      ref.lengths[2] + 2 * zone.captureRange,
    );
    const shellColor = Cesium.Color.fromBytes(
      ref.color[0], ref.color[1], ref.color[2],
      Math.round(0.15 * 255),
    );
    const ent = viewer.entities.add({
      position: center,
      orientation,
      box: {
        dimensions: expanded,
        material: shellColor,
        outline: true,
        outlineColor: Cesium.Color.fromBytes(ref.color[0], ref.color[1], ref.color[2], 200),
        outlineWidth: 1,
      },
    });
    geometryEntities.push(ent);
  }

  viewer.scene.requestRender();
}
```

Also add the import for `GeometryShape` at the top alongside `MissionGeometry`:

```typescript
import type { FrameDto, Origin, MissionGeometry, GeometryShape } from '../types';
```

- [ ] **Step 3: Wire `setGeometry` into the `ViewerHandle` returned by the effect**

In the `handle: ViewerHandle = { ... }` literal (around line 127-205), add a `setGeometry` method. Insert after `setOrigin`:

```typescript
      setGeometry: (geometry) => {
        renderGeometry(geometry, viewer, geometryEntitiesRef.current, originRef.current);
      },
```

- [ ] **Step 4: Clean up geometry entities in the effect cleanup**

In the effect's `return () => { ... }` cleanup (around line 208-218), add geometry cleanup. Insert after `targetLineRef.current = null;`:

```typescript
      for (const ent of geometryEntitiesRef.current) viewer.entities.remove(ent);
      geometryEntitiesRef.current = [];
```

- [ ] **Step 5: Verify TypeScript compiles**

```bash
docker compose exec web npm run build
# OR
docker compose logs --tail 50 web
```

Expected: no type errors. Cesium types should be happy with `BoxGraphics` / `EllipsoidGraphics` constructors.

- [ ] **Step 6: Commit**

```bash
git add webapp/web/src/components/CesiumViewer.tsx
git commit -m "feat(viewer): render static mission geometry as box/sphere primitives"
```

---

## Task 5: Plumb geometry from start response into the viewer

**Files:**
- Modify: `webapp/web/src/AppShell.tsx` (`handleStarted` and `handleStopped`)

- [ ] **Step 1: Pass geometry to viewer in `handleStarted`**

In `webapp/web/src/AppShell.tsx`, modify `handleStarted` (around line 77-83). Replace the existing function with:

```typescript
  function handleStarted(resp: MissionStartResponse) {
    viewerRef.current?.setOrigin(resp.origin);
    viewerRef.current?.setGeometry(resp.geometry ?? null);
    setOrigin(resp.origin);
    setSelectedEntityId(null);
    setLastReport(null);
    stopHandledRef.current = false;
  }
```

(Order matters: `setOrigin` first so the viewer has the frame origin established before geometry tries to project from it.)

- [ ] **Step 2: Clear geometry in `handleStopped`**

Modify `handleStopped` (around line 84-94). Add the clear call at the top of the function body, before the dedup guard returns:

```typescript
  async function handleStopped() {
    if (stopHandledRef.current) return;
    stopHandledRef.current = true;
    viewerRef.current?.setGeometry(null);
    try {
      const lines = await fetchReport();
      if (lines.length > 0) {
        setLastReport(lines);
        setReportOpen(true);
      }
    } catch { /* ignore */ }
  }
```

- [ ] **Step 3: Verify build still passes**

```bash
docker compose exec web npm run build
```

Expected: no type errors. `resp.geometry` is `MissionGeometry | undefined` from Task 3.

- [ ] **Step 4: Commit**

```bash
git add webapp/web/src/AppShell.tsx
git commit -m "feat(app): wire mission geometry from start response into viewer"
```

---

## Task 6: End-to-end manual verification

No automated tests for v1 (per spec §7). This task is the final acceptance check.

- [ ] **Step 1: Bring up the full stack**

```bash
docker compose up --build
```

Wait for all three services (scrimmage, api, web) to log "ready" / "Listening". Open `http://localhost:5173` (or whichever port `web` exposes).

- [ ] **Step 2: Start `capture-the-flag.xml` and visually verify**

Pick `capture-the-flag.xml` from the dropdown, click ▶ Start. Expected to see in the Cesium viewer:

- Two opaque-faced cubes: one blue, one red, each ~500m on a side, sitting symmetric across the origin.
- Two small colored spheres (flag positions), one inside each boundary.
- Two larger faint translucent shells around each boundary cube (capture zones, ~10m larger on each axis from `capture_range="5"`).
- Drones spawn and move within the boundaries on top of the geometry.

If the blue boundary appears WEST of origin instead of east (or any other axis swap): the local frame is NED, not ENU. Fix in `renderGeometry` by swapping center/lengths components when calling `enuToCartesian` (e.g., for NED: `enuToCartesian(origin, center[1], center[0], -center[2])`). Do the same swap for `lengths`. Verify and commit the fix.

- [ ] **Step 3: Verify clean swap to a no-geometry mission**

Click ■ Stop. Verify the boundaries and shells disappear immediately.

Pick `predator_prey_boids.xml`, click ▶ Start. Verify:
- No geometry renders (mission has no `entity_interaction` shapes).
- Drones still appear and move normally.
- Browser console has no errors related to geometry.

- [ ] **Step 4: Verify no regression on existing features**

While `predator_prey_boids` is running, verify:
- Entity selection (sidebar click + viewport click) still works.
- Recenter button works.
- Pause / Resume works.
- Stop produces the report modal.

- [ ] **Step 5: Commit any verification fixes**

If Step 2 required a coordinate-frame swap or any other small fix:

```bash
git add webapp/web/src/components/CesiumViewer.tsx
git commit -m "fix(viewer): correct geometry coordinate frame after E2E verification"
```

If everything worked first try, this step is a no-op.

- [ ] **Step 6: Update project memory and README**

Quick update to `webapp/README.md` Future Work section: remove the "Static mission geometry" bullet from line ~181, since it's now implemented.

```bash
# Manual edit: delete or update the bullet at webapp/README.md:181
git add webapp/README.md
git commit -m "docs: remove static mission geometry from future-work list"
```

---

## Self-review (post-write check, completed inline)

**Spec coverage:**

| Spec section | Implementation |
|---|---|
| §2 In scope: cuboid + sphere shapes | Task 1 parser, Task 4 renderer |
| §2 In scope: capture-zone shells (option B) | Task 1 parser CaptureInBoundaryInteraction handling, Task 4 shell rendering with `lengths + 2*captureRange` |
| §2 In scope: one-shot push at mission start | Task 2 wires into `/missions/start` response, Task 5 reads on `handleStarted` |
| §2 In scope: render with color/opacity from XML | Task 1 parses color/opacity attrs, Task 4 uses `Color.fromBytes` |
| §2 In scope: clear on stop / new mission | Task 5 `handleStopped` calls `setGeometry(null)`; Task 4 `renderGeometry` clears existing entities at start |
| §2 Out of scope: skip unknown types | Task 1: `if shape_type in ("cuboid", "sphere")` filters silently |
| §6 Error: malformed XML | Task 1: try/except returns empty lists |
| §6 Error: unknown shape type | Task 1: skip without logging spam (it's expected for non-renderable interaction tags like `EnforceBoundaryInteraction`, `SimpleCollision`) |
| §6 Error: missing required attribute | Task 1: explicit checks for `lengths` (cuboid) and `radius` (sphere) with logged skip |
| §6 Error: capture zone refs unknown boundary | Task 4: `console.warn` and skip shell |
| §6 Error: geometry arrives before origin | Task 5 calls `setOrigin` before `setGeometry`; Task 4 `renderGeometry` returns early if `!origin` |
| §7 Manual verification: capture-the-flag, predator_prey_boids, mission swap, browser reload | Task 6 covers these |

**Placeholder scan:** No TBDs, TODOs, vague handwaving, or "similar to above" references. Every code block is complete.

**Type consistency:** `MissionGeometry`, `GeometryShape`, `CaptureZone` defined in Task 3, used identically in Task 4 and Task 5. Python field names (`shapes`, `captureZones`, `kind`, `teamId`, `boundaryId`, `captureRange`) match TypeScript field names exactly (camelCase, since Python emits them that way in the dict).

**Scope check:** Single feature, ~1.5 hours, six bite-sized tasks. Not a candidate for decomposition.

**Late-join handling:** Deferred per spec amendment — frontend reads geometry from start response, so a browser tab that connects mid-mission won't see geometry until next Start. Acceptable for hackathon (single-operator, single-tab demo).

**Reverse-proxy / CORS / SignalR concerns:** None — we're using existing pass-through endpoints with no new transport.
