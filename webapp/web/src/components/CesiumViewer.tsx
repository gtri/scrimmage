import { useEffect, useRef, type MutableRefObject } from 'react';
import * as Cesium from 'cesium';
import type { FrameDto, Origin } from '../types';
import { enuToCartesian } from '../lib/enuToCartesian';
import { predatorTarget } from '../lib/predatorTarget';

const ION_TOKEN = (import.meta as any).env.VITE_CESIUM_ION_TOKEN as string | undefined;

export interface ProjectedPoint {
  x: number;
  y: number;
  visible: boolean;
}

export interface ViewerHandle {
  applyFrame: (frame: FrameDto) => void;
  setOrigin: (origin: Origin | null) => void;
  /**
   * Inform the viewer of the operator-issued target assignment so the
   * predator→target polyline and chain-track logic honor it (instead of
   * always picking nearest prey by heuristic). Pass null to clear.
   */
  setAssignedTargetId: (id: number | null) => void;
  /** Operator-triggered: fit all current entities in view. No-op if no entities. */
  recenter: () => void;
  /** Programmatically select an entity (or clear with null). Mirrors Cesium's selectionIndicator + infoBox. */
  selectEntity: (id: number | null) => void;
  /**
   * Select the entity, fly the camera to it, and start tracking it.
   * No-op if the id isn't currently in the scene. The flyTo promise
   * rejects when the operator pans manually — that's expected and ignored.
   */
  flyToAndTrack: (id: number) => Promise<void>;
  /**
   * Subscribe to the screen-space projection of an entity. The callback fires
   * on each scene.postRender — i.e., whenever the camera or the entity moves.
   * Pass null to clear any active subscription. Returns a teardown fn.
   */
  subscribeProjection: (
    id: number | null,
    cb: (p: ProjectedPoint) => void,
  ) => () => void;
}

export interface ViewerProps {
  onReady?: (handle: ViewerHandle) => void;
  /** Fired when Cesium's selected entity changes — from a viewport click OR a programmatic selectEntity call. */
  onSelectionChanged?: (id: number | null) => void;
}

const TEAM_COLORS: Record<number, Cesium.Color> = {
  1: Cesium.Color.DODGERBLUE,
  2: Cesium.Color.CRIMSON,
};

export function CesiumViewer({ onReady, onSelectionChanged }: ViewerProps) {
  const ref = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Cesium.Viewer | null>(null);
  const entitiesRef = useRef<Map<number, Cesium.Entity>>(new Map());
  const originRef = useRef<Origin | null>(null);
  const selectionHandlerRef = useRef(onSelectionChanged);
  const projectionTeardownRef = useRef<(() => void) | null>(null);
  const targetLineRef = useRef<{ positions: Cesium.Cartesian3[] } | null>(null);
  const targetLineEntityRef = useRef<Cesium.Entity | null>(null);
  // Operator override fed in via handle.setAssignedTargetId; consumed inside
  // applyFrame so the polyline + chain-track logic honor the assignment.
  const assignedTargetIdRef = useRef<number | null>(null);
  selectionHandlerRef.current = onSelectionChanged;

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
      // Render only when something changes (camera, entities, time) — this is the
      // operator console, not a flight sim. Saves a lot of GPU when the camera is idle.
      requestRenderMode: true,
    });
    viewerRef.current = viewer;

    // Persistent target line — drawn each render via CallbackProperty so we
    // don't churn entities. Hidden when targetLineRef is null.
    targetLineEntityRef.current = viewer.entities.add({
      polyline: {
        positions: new Cesium.CallbackProperty(
          () => targetLineRef.current?.positions ?? [],
          false,
        ),
        show: new Cesium.CallbackProperty(
          () => targetLineRef.current != null,
          false,
        ) as unknown as Cesium.Property,
        width: 1.5,
        material: new Cesium.PolylineDashMaterialProperty({
          color: Cesium.Color.RED,
          dashLength: 16,
        }),
        arcType: Cesium.ArcType.NONE,
      },
    });

    if (ION_TOKEN) {
      Cesium.createWorldTerrainAsync()
        .then(t => { viewer.terrainProvider = t; viewer.scene.requestRender(); })
        .catch(err => console.warn('Cesium terrain load failed:', err));
    }

    // Initial camera — oblique view of the Camp Roberts area as a sensible default
    // before any mission is selected. setOrigin reframes for the actual mission origin
    // when the operator clicks Start. After mission start, the operator owns the camera.
    cameraLookAtOrigin(viewer, { lat: 35.721025, lon: -120.767925, alt: 300 });

    // Cesium widget needs to recompute its canvas size after the parent grid settles.
    // Without this, the credit bar and scene canvas can get clipped on first paint.
    const ro = new ResizeObserver(() => { viewer.resize(); viewer.scene.requestRender(); });
    ro.observe(ref.current);

    // Forward Cesium's selection event (fires on viewport click AND programmatic
    // viewer.selectedEntity assignment) up to the parent. The entity's id field
    // is set to the stringified numeric SCRIMMAGE entity id when we add it.
    const onCesiumSelection = (entity?: Cesium.Entity) => {
      const idStr = entity?.id;
      const id = idStr != null ? Number(idStr) : null;
      selectionHandlerRef.current?.(Number.isFinite(id as number) ? (id as number) : null);
    };
    viewer.selectedEntityChanged.addEventListener(onCesiumSelection);

    const handle: ViewerHandle = {
      applyFrame: (frame) => {
        applyFrame(
          frame, viewer, entitiesRef.current, originRef.current,
          targetLineRef, assignedTargetIdRef.current,
        );
        viewer.scene.requestRender(); // requestRenderMode requires explicit re-render after entity updates
      },
      setAssignedTargetId: (id) => {
        assignedTargetIdRef.current = id;
        // No requestRender — the next frame tick will pick up the new value.
      },
      setOrigin: (origin) => {
        originRef.current = origin;
        // Operator clicked Start — reframe the camera obliquely on the new mission's origin
        // so altitude differences between drones are visible immediately. This is operator-
        // implied, not automatic motion during a running sim.
        if (origin) {
          cameraLookAtOrigin(viewer, origin);
          viewer.scene.requestRender();
        }
      },
      recenter: () => {
        if (entitiesRef.current.size === 0) return;
        viewer.flyTo(viewer.entities, {
          duration: 1.0,
          offset: new Cesium.HeadingPitchRange(0, Cesium.Math.toRadians(-30), 0),
        }).catch(() => { /* operator can cancel by panning during the flight; not an error */ });
      },
      selectEntity: (id) => {
        const target = id == null ? undefined : entitiesRef.current.get(id);
        if (viewer.selectedEntity !== target) {
          viewer.selectedEntity = target;
          viewer.scene.requestRender();
        }
      },
      flyToAndTrack: async (id) => {
        const target = entitiesRef.current.get(id);
        if (!target) return;
        viewer.selectedEntity = target;
        try {
          await viewer.flyTo(target, { duration: 0.8 });
          viewer.trackedEntity = target;
        } catch {
          // Operator cancelled the flight by panning. Not an error.
        }
      },
      subscribeProjection: (id, cb) => {
        // Tear down any existing subscription first — only one at a time.
        projectionTeardownRef.current?.();
        projectionTeardownRef.current = null;

        if (id == null) {
          cb({ x: 0, y: 0, visible: false });
          return () => {};
        }

        const listener = () => {
          const ent = entitiesRef.current.get(id);
          if (!ent || !ent.position) { cb({ x: 0, y: 0, visible: false }); return; }
          const cartesian = ent.position.getValue(viewer.clock.currentTime);
          if (!cartesian) { cb({ x: 0, y: 0, visible: false }); return; }
          const win = Cesium.SceneTransforms.worldToWindowCoordinates(viewer.scene, cartesian);
          if (!win) { cb({ x: 0, y: 0, visible: false }); return; }
          const canvas = viewer.scene.canvas;
          const visible =
            win.x >= 0 && win.x < canvas.clientWidth &&
            win.y >= 0 && win.y < canvas.clientHeight;
          cb({ x: win.x, y: win.y, visible });
        };

        viewer.scene.postRender.addEventListener(listener);

        const teardown = () => {
          viewer.scene.postRender.removeEventListener(listener);
          projectionTeardownRef.current = null;
        };
        projectionTeardownRef.current = teardown;

        listener(); // fire once so the card appears immediately on selection
        // Force a render so postRender fires at least once if the camera is idle.
        viewer.scene.requestRender();

        return teardown;
      },
    };
    onReady?.(handle);

    return () => {
      projectionTeardownRef.current?.();
      viewer.selectedEntityChanged.removeEventListener(onCesiumSelection);
      ro.disconnect();
      targetLineEntityRef.current = null;
      targetLineRef.current = null;
      viewer.destroy();
      viewerRef.current = null;
      entitiesRef.current.clear();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Absolute positioning inside a position:relative parent guarantees the Cesium widget
  // exactly fills the cell regardless of grid sizing quirks.
  return <div ref={ref} style={{ position: 'absolute', inset: 0 }} />;
}

/**
 * Position the camera in an oblique 3D view of the given origin point.
 * heading=0 puts the camera due north of the origin looking south; pitch=-30°
 * gives a perspective view where vertical separation between drones is visible.
 * range=2500m fits a typical mission's spawn spread (~500-1000m radius).
 *
 * Cesium's lookAt sets a reference frame transform; we reset it to identity
 * immediately so subsequent operator pan/zoom/tilt behave normally (otherwise
 * the camera would orbit the target instead of moving freely).
 */
function cameraLookAtOrigin(viewer: Cesium.Viewer, origin: Origin) {
  viewer.camera.lookAt(
    Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt),
    new Cesium.HeadingPitchRange(
      0,
      Cesium.Math.toRadians(-30),
      2500
    )
  );
  viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
}

function applyFrame(
  frame: FrameDto,
  viewer: Cesium.Viewer,
  entities: Map<number, Cesium.Entity>,
  origin: Origin | null,
  targetLineRef: MutableRefObject<{ positions: Cesium.Cartesian3[] } | null>,
  assignedTargetId: number | null,
) {
  if (!origin) return; // can't render without origin

  const seen = new Set<number>();
  for (const e of frame.entities) {
    if (!e.active) {
      const existing = entities.get(e.id);
      if (existing) { viewer.entities.remove(existing); entities.delete(e.id); }
      continue;
    }
    seen.add(e.id);
    const pos = enuToCartesian(origin, e.position.x, e.position.y, e.position.z);
    const color = TEAM_COLORS[e.teamId] ?? Cesium.Color.GRAY;
    let ent = entities.get(e.id);
    if (!ent) {
      ent = viewer.entities.add({
        // Stringified SCRIMMAGE entity id — lets the selection event handler
        // recover the numeric id, and lets us look up by id from a sidebar click.
        id: String(e.id),
        name: `Entity #${e.id} · team ${e.teamId}`,
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

  // Update predator-target polyline. Same selector the badge uses — including
  // the operator override so the line follows the assigned target, not just
  // whatever's nearest.
  const pair = predatorTarget(frame, assignedTargetId);
  if (pair) {
    const predEnt = entities.get(pair.predatorId);
    const preyEnt = entities.get(pair.targetId);
    if (predEnt && preyEnt && predEnt.position && preyEnt.position) {
      const t = viewer.clock.currentTime;
      const a = predEnt.position.getValue(t);
      const b = preyEnt.position.getValue(t);
      if (a && b) {
        targetLineRef.current = { positions: [a, b] };
      } else {
        targetLineRef.current = null;
      }
    } else {
      targetLineRef.current = null;
    }
  } else {
    targetLineRef.current = null;
  }

  // Chain-track: when the operator was tracking an entity that just
  // disappeared (captured), follow the predator's new current target so
  // the camera follows the hunt. Clears tracking only if no new target.
  const tracked = viewer.trackedEntity;
  if (tracked && tracked.id) {
    const trackedId = Number(tracked.id);
    if (Number.isFinite(trackedId) && !seen.has(trackedId)) {
      const next = predatorTarget(frame, assignedTargetId);
      const nextEnt = next ? entities.get(next.targetId) : undefined;
      if (nextEnt) {
        viewer.trackedEntity = nextEnt;
        viewer.selectedEntity = nextEnt;
      } else {
        viewer.trackedEntity = undefined;
      }
    }
  }
}
