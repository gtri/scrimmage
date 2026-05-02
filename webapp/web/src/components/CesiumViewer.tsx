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
