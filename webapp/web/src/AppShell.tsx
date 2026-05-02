import { useRef, useState } from 'react';
import { CesiumViewer, type ViewerHandle } from './components/CesiumViewer';
import { MissionPicker } from './components/MissionPicker';
import { EntityList } from './components/EntityList';
import { Placeholder } from './components/panels/Placeholder';
import { HelpOverlay } from './components/HelpOverlay';
import { useFrameStream } from './hooks/useFrameStream';
import type { MissionStartResponse, Origin } from './types';

export function AppShell() {
  const viewerRef = useRef<ViewerHandle | null>(null);
  const [origin, setOrigin] = useState<Origin | null>(null);

  const { connected, latestFrame } = useFrameStream(frame => {
    viewerRef.current?.applyFrame(frame);
  });

  function handleStarted(resp: MissionStartResponse) {
    viewerRef.current?.setOrigin(resp.origin);
    setOrigin(resp.origin);
  }
  function handleStopped() {
    // Leave the last frame on screen until the next start; nothing to do here.
  }
  function handleRecenter() {
    viewerRef.current?.recenter();
  }

  return (
    <div style={{ display: 'grid', gridTemplateRows: 'auto 1fr', height: '100vh', background: '#111', color: '#eee' }}>
      {/* Header */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 16, flexWrap: 'wrap' }}>
        <MissionPicker onStarted={handleStarted} onStopped={handleStopped} />
        {origin && <OriginBadge origin={origin} />}
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
          <RecenterButton onClick={handleRecenter} disabled={!latestFrame || latestFrame.entities.length === 0} />
          <HelpOverlay />
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

function OriginBadge({ origin }: { origin: Origin }) {
  return (
    <span
      title="Mission geographic origin (latitude, longitude, altitude)"
      style={{
        fontFamily: 'ui-monospace, Menlo, Consolas, monospace', fontSize: 12,
        color: '#bbb', padding: '2px 8px', border: '1px solid #444', borderRadius: 4,
      }}
    >
      📍 {origin.lat.toFixed(4)}, {origin.lon.toFixed(4)} · {origin.alt}m
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
