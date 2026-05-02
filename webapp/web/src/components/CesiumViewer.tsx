import { useEffect, useRef } from 'react';
import * as Cesium from 'cesium';
import type { FrameDto, Origin } from '../types';
import { enuToCartesian } from '../lib/enuToCartesian';

const ION_TOKEN = (import.meta as any).env.VITE_CESIUM_ION_TOKEN as string | undefined;

export interface ViewerHandle {
  applyFrame: (frame: FrameDto) => void;
  setOrigin: (origin: Origin | null) => void;
  /** Operator-triggered: fit all current entities in view. No-op if no entities. */
  recenter: () => void;
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
      // Render only when something changes (camera, entities, time) — this is the
      // operator console, not a flight sim. Saves a lot of GPU when the camera is idle.
      requestRenderMode: true,
    });
    viewerRef.current = viewer;

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

    const handle: ViewerHandle = {
      applyFrame: (frame) => {
        applyFrame(frame, viewer, entitiesRef.current, originRef.current);
        viewer.scene.requestRender(); // requestRenderMode requires explicit re-render after entity updates
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
    };
    onReady?.(handle);

    return () => {
      ro.disconnect();
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
