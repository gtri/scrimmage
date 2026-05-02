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
