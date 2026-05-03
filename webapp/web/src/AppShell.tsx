import { useEffect, useMemo, useRef, useState } from 'react';
import { CesiumViewer, type ViewerHandle } from './components/CesiumViewer';
import { MissionPicker } from './components/MissionPicker';
import { EntityList } from './components/EntityList';
import { Placeholder } from './components/panels/Placeholder';
import { TopicFeedPanel } from './components/panels/TopicFeedPanel';
import { HelpOverlay } from './components/HelpOverlay';
import { ReportModal } from './components/ReportModal';
import { useFrameStream } from './hooks/useFrameStream';
import { EntityInspectorCard } from './components/EntityInspectorCard';
import { useCapturedState } from './hooks/useCapturedState';
import type { ProjectedPoint } from './components/CesiumViewer';
import { reverseGeocode } from './lib/geocode';
import { fetchReport } from './lib/api';
import type { MissionStartResponse, Origin } from './types';
import { PredatorTargetBadge } from './components/PredatorTargetBadge';
import { predatorTarget } from './lib/predatorTarget';

export function AppShell() {
  const viewerRef = useRef<ViewerHandle | null>(null);
  const [origin, setOrigin] = useState<Origin | null>(null);
  const [selectedEntityId, setSelectedEntityId] = useState<number | null>(null);
  const [lastReport, setLastReport] = useState<string[] | null>(null);
  const [reportOpen, setReportOpen] = useState(false);
  // Dedup guard: handleStopped can fire twice in rapid succession when the operator
  // clicks Stop AND the status poller sees idle inside the same ~500ms window.
  const stopHandledRef = useRef(false);

  const { connected, latestFrame } = useFrameStream(frame => {
    viewerRef.current?.applyFrame(frame);
  });

  const [screenPosition, setScreenPosition] = useState<ProjectedPoint | null>(null);
  const inspector = useCapturedState(
    selectedEntityId,
    latestFrame,
    () => {
      setSelectedEntityId(null);
      viewerRef.current?.selectEntity(null);
    },
  );

  const target = useMemo(
    () => latestFrame ? predatorTarget(latestFrame) : null,
    [latestFrame],
  );

  useEffect(() => {
    const teardown = viewerRef.current?.subscribeProjection(
      selectedEntityId,
      setScreenPosition,
    );
    if (selectedEntityId == null) setScreenPosition(null);
    return () => { teardown?.(); };
  }, [selectedEntityId]);

  function handleStarted(resp: MissionStartResponse) {
    viewerRef.current?.setOrigin(resp.origin);
    setOrigin(resp.origin);
    setSelectedEntityId(null);
    setLastReport(null);
    stopHandledRef.current = false;
  }
  async function handleStopped() {
    if (stopHandledRef.current) return;
    stopHandledRef.current = true;
    try {
      const lines = await fetchReport();
      if (lines.length > 0) {
        setLastReport(lines);
        setReportOpen(true);
      }
    } catch { /* ignore */ }
  }
  function handleRecenter() { viewerRef.current?.recenter(); }
  function handleSelectFromList(id: number | null) {
    setSelectedEntityId(id);
    viewerRef.current?.selectEntity(id);
  }
  function handleSelectionFromViewport(id: number | null) {
    setSelectedEntityId(id);
  }

  return (
    <div style={{
      display: 'grid', gridTemplateRows: 'auto 1fr', height: '100vh',
      background: 'var(--bg-deepest)',
    }}>
      {/* Header */}
      <header style={{
        display: 'flex', alignItems: 'center', gap: 16, flexWrap: 'wrap',
        padding: '8px 16px',
        background: 'var(--bg-header)',
        borderBottom: '1px solid var(--border-default)',
      }}>
        <Brand />
        <MissionPicker onStarted={handleStarted} onStopped={handleStopped} />
        {origin && <OriginBadge origin={origin} />}
        {lastReport && (
          <button onClick={() => setReportOpen(true)} title="Re-open the most recent mission report">
            ▤ Last report
          </button>
        )}
        <span style={{ flex: 1 }} />
        <StreamBadge connected={connected} />
      </header>

      {/* Three-column body: sidebar | viewer | right rail */}
      <div style={{ display: 'grid', gridTemplateColumns: '240px 1fr 280px', minHeight: 0 }}>
        <aside style={{
          background: 'var(--bg-panel)', overflowY: 'auto',
          borderRight: '1px solid var(--border-default)',
        }}>
          <PanelHeader>Entities</PanelHeader>
          <EntityList
            frame={latestFrame}
            selectedId={selectedEntityId}
            onSelect={handleSelectFromList}
          />
        </aside>

        <main style={{ position: 'relative', background: 'var(--bg-base)' }}>
          <CesiumViewer
            onReady={h => { viewerRef.current = h; }}
            onSelectionChanged={handleSelectionFromViewport}
          />
          <RecenterButton onClick={handleRecenter} disabled={!latestFrame || latestFrame.entities.length === 0} />
          <PredatorTargetBadge
            target={target}
            onClick={(id) => { void viewerRef.current?.flyToAndTrack(id); }}
          />
          <HelpOverlay />
          {origin && inspector.entity && inspector.mode !== 'idle' && (
            <EntityInspectorCard
              entity={inspector.entity}
              origin={origin}
              screenPosition={screenPosition}
              mode={inspector.mode}
              killerId={inspector.killerId}
            />
          )}
        </main>

        <aside style={{
          background: 'var(--bg-panel)', overflowY: 'auto',
          borderLeft: '1px solid var(--border-default)',
        }}>
          <Placeholder title="Commands" description="Operator commands (target_assignment, swap_team) — coming in v2." />
          <TopicFeedPanel />
          <Placeholder title="Tags" description="Annotate entities with operator-defined labels — v2." />
        </aside>
      </div>

      {reportOpen && lastReport && (
        <ReportModal lines={lastReport} onClose={() => setReportOpen(false)} />
      )}
    </div>
  );
}

function Brand() {
  return (
    <div style={{
      display: 'flex', alignItems: 'baseline', gap: 6,
      paddingRight: 12, marginRight: 4,
      borderRight: '1px solid var(--border-default)',
    }}>
      <span style={{
        fontWeight: 700, fontSize: 18, letterSpacing: '0.18em',
        color: 'var(--accent)', textTransform: 'uppercase',
      }}>SCRIMMAGE</span>
      <span style={{
        fontSize: 11, letterSpacing: '0.25em', color: 'var(--text-muted)',
        textTransform: 'uppercase',
      }}>C2</span>
    </div>
  );
}

function PanelHeader({ children }: { children: React.ReactNode }) {
  return (
    <div style={{
      padding: '8px 12px',
      fontSize: 10, fontWeight: 700, letterSpacing: '0.2em',
      textTransform: 'uppercase', color: 'var(--text-secondary)',
      borderBottom: '1px solid var(--border-default)',
      background: 'linear-gradient(180deg, rgba(255,255,255,0.02), transparent)',
    }}>{children}</div>
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
        display: 'inline-flex', alignItems: 'center', gap: 8,
        fontFamily: 'var(--font-mono)', fontSize: 11,
        color: 'var(--text-secondary)',
        padding: '4px 10px',
        border: '1px solid var(--border-default)',
        borderRadius: 2,
        background: 'var(--bg-base)',
      }}
    >
      <span style={{ color: 'var(--accent)' }}>◉</span>
      {locationName && <span style={{ color: 'var(--text-primary)', fontFamily: 'var(--font-ui)', fontWeight: 600, letterSpacing: '0.05em' }}>{locationName}</span>}
      <span>{origin.lat.toFixed(4)}, {origin.lon.toFixed(4)} · {origin.alt}m</span>
    </span>
  );
}

function StreamBadge({ connected }: { connected: boolean }) {
  return (
    <span style={{
      display: 'inline-flex', alignItems: 'center', gap: 6,
      fontSize: 11, fontWeight: 600, textTransform: 'uppercase', letterSpacing: '0.15em',
      color: connected ? 'var(--accent-ok)' : 'var(--accent-danger)',
    }}>
      <span style={{
        width: 8, height: 8, borderRadius: '50%',
        background: connected ? 'var(--accent-ok)' : 'var(--accent-danger)',
        boxShadow: connected ? '0 0 8px var(--accent-ok)' : '0 0 8px var(--accent-danger)',
      }} />
      Stream {connected ? 'online' : 'offline'}
    </span>
  );
}

function RecenterButton({ onClick, disabled }: { onClick: () => void; disabled: boolean }) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      title="Recenter camera on current entities"
      style={{ position: 'absolute', top: 8, left: 8, zIndex: 10 }}
    >
      ⊕ Recenter
    </button>
  );
}
