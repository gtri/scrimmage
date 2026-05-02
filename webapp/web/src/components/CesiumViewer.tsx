import { useEffect, useRef, type MutableRefObject } from 'react';
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
  // Tracks whether we've auto-framed the camera on the entities for the current mission.
  // Reset on setOrigin so each Start triggers a fresh fit-to-entities flight.
  const framedRef = useRef(false);

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

    // Default view (overwritten once first frame arrives via flyTo(entities))
    viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(-120.767925, 35.721025, 5000),
    });

    const handle: ViewerHandle = {
      applyFrame: (frame) => applyFrame(frame, viewer, entitiesRef.current, originRef.current, framedRef),
      setOrigin: (origin) => {
        originRef.current = origin;
        framedRef.current = false; // re-frame on next frame
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
  origin: Origin | null,
  framedRef: MutableRefObject<boolean>
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

  // First frame for this mission with at least one entity → fit them all in view, once.
  // Cesium computes the bounding sphere of all viewer.entities and flies to fit it.
  // Subsequent frames are silent — user can pan/zoom freely and the camera stays put.
  if (!framedRef.current && entities.size > 0) {
    framedRef.current = true;
    viewer.flyTo(viewer.entities, {
      duration: 1.5,
      offset: new Cesium.HeadingPitchRange(0, Cesium.Math.toRadians(-55), 0),
    }).catch(() => { /* user-initiated camera move can cancel the flight; not an error */ });
  }
}
