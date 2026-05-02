import { useEffect, useRef, useState } from 'react';
import { CesiumViewer, type ViewerHandle } from './components/CesiumViewer';
import { MissionPicker } from './components/MissionPicker';
import { EntityList } from './components/EntityList';
import { Placeholder } from './components/panels/Placeholder';
import { HelpOverlay } from './components/HelpOverlay';
import { ReportModal } from './components/ReportModal';
import { useFrameStream } from './hooks/useFrameStream';
import { reverseGeocode } from './lib/geocode';
import { fetchReport } from './lib/api';
import type { MissionStartResponse, Origin } from './types';

export function AppShell() {
  const viewerRef = useRef<ViewerHandle | null>(null);
  const [origin, setOrigin] = useState<Origin | null>(null);
  const [selectedEntityId, setSelectedEntityId] = useState<number | null>(null);
  const [lastReport, setLastReport] = useState<string[] | null>(null);
  const [reportOpen, setReportOpen] = useState(false);

  const { connected, latestFrame } = useFrameStream(frame => {
    viewerRef.current?.applyFrame(frame);
  });

  function handleStarted(resp: MissionStartResponse) {
    viewerRef.current?.setOrigin(resp.origin);
    setOrigin(resp.origin);
    setSelectedEntityId(null); // fresh mission → drop any prior selection
    setLastReport(null);       // fresh mission → previous report is now stale
  }
  async function handleStopped() {
    // Mission stopped — fetch the report scrimmage emitted on shutdown and surface it.
    try {
      const lines = await fetchReport();
      if (lines.length > 0) {
        setLastReport(lines);
        setReportOpen(true);
      }
    } catch {
      // Network failure — don't block; operator can re-run if they want.
    }
  }
  function handleRecenter() {
    viewerRef.current?.recenter();
  }
  function handleSelectFromList(id: number | null) {
    setSelectedEntityId(id);
    viewerRef.current?.selectEntity(id);
  }
  function handleSelectionFromViewport(id: number | null) {
    setSelectedEntityId(id);
  }

  return (
    <div style={{ display: 'grid', gridTemplateRows: 'auto 1fr', height: '100vh', background: '#111', color: '#eee' }}>
      {/* Header */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 16, flexWrap: 'wrap' }}>
        <MissionPicker onStarted={handleStarted} onStopped={handleStopped} />
        {origin && <OriginBadge origin={origin} />}
        {lastReport && (
          <button
            onClick={() => setReportOpen(true)}
            title="Re-open the most recent mission report"
            style={{
              padding: '4px 10px', fontSize: 12, color: '#ddd',
              background: 'transparent', border: '1px solid #555', borderRadius: 4,
              cursor: 'pointer',
            }}
          >📊 Last report</button>
        )}
        <span style={{ color: connected ? '#7f7' : '#f77', marginRight: 16 }}>
          ● Stream {connected ? 'connected' : 'disconnected'}
        </span>
      </div>

      {/* Three-column body: sidebar | viewer | right rail */}
      <div style={{ display: 'grid', gridTemplateColumns: '240px 1fr 280px', minHeight: 0 }}>
        <aside style={{ background: '#1a1a1a', overflowY: 'auto', borderRight: '1px solid #333' }}>
          <div style={{ padding: 8, color: '#888', fontSize: 11, textTransform: 'uppercase' }}>Entities</div>
          <EntityList
            frame={latestFrame}
            selectedId={selectedEntityId}
            onSelect={handleSelectFromList}
          />
        </aside>

        <main style={{ position: 'relative' }}>
          <CesiumViewer
            onReady={h => { viewerRef.current = h; }}
            onSelectionChanged={handleSelectionFromViewport}
          />
          <RecenterButton onClick={handleRecenter} disabled={!latestFrame || latestFrame.entities.length === 0} />
          <HelpOverlay />
        </main>

        <aside style={{ background: '#1a1a1a', overflowY: 'auto', borderLeft: '1px solid #333' }}>
          <Placeholder title="Commands" description="Operator commands (target_assignment, swap_team) — coming in v2." />
          <Placeholder title="Topic Stream" description="Subscribe to a drone's pub/sub topics — needs a SCRIMMAGE TopicTap plugin (v2)." />
          <Placeholder title="Tags" description="Annotate entities with operator-defined labels — v2." />
        </aside>
      </div>

      {reportOpen && lastReport && (
        <ReportModal lines={lastReport} onClose={() => setReportOpen(false)} />
      )}
    </div>
  );
}

function OriginBadge({ origin }: { origin: Origin }) {
  const [locationName, setLocationName] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    setLocationName(null);
    reverseGeocode(origin.lat, origin.lon).then(name => {
      if (!cancelled) setLocationName(name);
    });
    return () => { cancelled = true; };
  }, [origin.lat, origin.lon]);

  return (
    <span
      title="Mission geographic origin (latitude, longitude, altitude)"
      style={{
        fontFamily: 'ui-monospace, Menlo, Consolas, monospace', fontSize: 12,
        color: '#bbb', padding: '2px 8px', border: '1px solid #444', borderRadius: 4,
      }}
    >
      📍 {locationName && <span style={{ color: '#eee' }}>{locationName} · </span>}
      {origin.lat.toFixed(4)}, {origin.lon.toFixed(4)} · {origin.alt}m
    </span>
  );
}

function RecenterButton({ onClick, disabled }: { onClick: () => void; disabled: boolean }) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      title="Recenter camera on current entities"
      style={{
        position: 'absolute', top: 8, left: 8, zIndex: 10,
        padding: '4px 10px',
        background: 'rgba(20,20,20,0.85)', color: disabled ? '#666' : '#ddd',
        border: '1px solid #555', borderRadius: 4,
        fontSize: 12, cursor: disabled ? 'not-allowed' : 'pointer',
      }}
    >
      ⊕ Recenter
    </button>
  );
}
