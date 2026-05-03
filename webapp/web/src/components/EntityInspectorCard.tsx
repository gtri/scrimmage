import { useEffect, useState } from 'react';
import type { EntityDto, FrameDto } from '../types';
import type { Origin } from '../lib/enuToCartesian';
import {
  speed,
  headingDeg,
  climbRateMps,
  eulerFromQuat,
  enuToGeo,
} from '../lib/deriveEntityStats';
import type { ProjectedPoint } from './CesiumViewer';
import { assignTarget, clearTarget } from '../lib/commandsApi';

export interface EntityInspectorCardProps {
  entity: EntityDto;
  origin: Origin;
  screenPosition: ProjectedPoint | null;
  mode: 'live' | 'captured';
  killerId?: number | null;
  latestFrame?: FrameDto | null;
  // Lifted to AppShell so PredatorTargetBadge + camera tracking can also see the
  // operator's override. Card is the source of intent (button click), AppShell
  // is the source of truth (also runs the auto-clear when target leaves frame).
  assignedTargetId?: number | null;
  onAssignedTargetIdChange?: (id: number | null) => void;
}

const CARD_WIDTH = 240;
const CARD_OFFSET_X = 16;
const CARD_OFFSET_Y = -120;

export function EntityInspectorCard({
  entity,
  origin,
  screenPosition,
  mode,
  killerId,
  latestFrame,
  assignedTargetId = null,
  onAssignedTargetIdChange,
}: EntityInspectorCardProps) {
  // Pick the first team-2 entity as "the predator." v1: multi-predator deferred.
  const predatorEntityId = latestFrame?.entities.find(e => e.teamId === 2)?.id ?? null;

  const [commandError, setCommandError] = useState<string | null>(null);

  // Auto-clear error after 3s.
  useEffect(() => {
    if (commandError == null) return;
    const t = window.setTimeout(() => setCommandError(null), 3000);
    return () => window.clearTimeout(t);
  }, [commandError]);

  if (!screenPosition || !screenPosition.visible) return null;

  const showTargetButton =
    mode === 'live' && entity.teamId === 1 && predatorEntityId != null;

  const v = entity.velocity;
  const sp = speed(v);
  const heading = headingDeg(v);
  const climb = climbRateMps(v);
  const euler = eulerFromQuat(entity.orientation);
  const geo = enuToGeo(origin, entity.position.x, entity.position.y, entity.position.z);

  const teamColor =
    entity.teamId === 1 ? 'var(--team-blue)' :
    entity.teamId === 2 ? 'var(--team-red)' :
    'var(--text-secondary)';

  const left = screenPosition.x + CARD_OFFSET_X;
  const top = screenPosition.y + CARD_OFFSET_Y;
  const captured = mode === 'captured';

  return (
    <>
    <svg
      style={{
        position: 'absolute',
        left: 0,
        top: 0,
        width: '100%',
        height: '100%',
        pointerEvents: 'none',
        zIndex: 19,
        overflow: 'visible',
      }}
    >
      <line
        x1={screenPosition.x}
        y1={screenPosition.y}
        x2={screenPosition.x + CARD_OFFSET_X}
        y2={screenPosition.y + CARD_OFFSET_Y}
        stroke={captured ? 'var(--accent-danger)' : teamColor}
        strokeWidth={1}
        strokeOpacity={0.7}
      />
    </svg>
    <div
      style={{
        position: 'absolute', left, top, width: CARD_WIDTH,
        background: 'var(--bg-panel)',
        border: `1px solid ${captured ? 'var(--accent-danger)' : 'var(--border-default)'}`,
        borderLeft: `3px solid ${teamColor}`,
        borderRadius: 2,
        boxShadow: '0 4px 12px rgba(0,0,0,0.4)',
        color: 'var(--text-primary)',
        fontFamily: 'var(--font-mono)', fontSize: 11,
        zIndex: 20,
        pointerEvents: showTargetButton ? 'auto' : 'none',
        opacity: captured ? 0.85 : 1,
      }}
    >
      {captured && (
        <div style={{
          padding: '4px 8px',
          background: 'var(--accent-danger)',
          color: 'white',
          fontWeight: 700, letterSpacing: '0.1em',
          textAlign: 'center',
        }}>
          CAPTURED{killerId != null ? ` BY #${killerId}` : ''}
        </div>
      )}
      <div style={{
        padding: '6px 10px',
        display: 'flex', alignItems: 'center', gap: 8,
        borderBottom: '1px solid var(--border-default)',
      }}>
        <span style={{ color: teamColor, fontWeight: 700 }}>#{entity.id}</span>
        <span style={{ color: 'var(--text-muted)' }}>·</span>
        <span style={{ color: 'var(--text-secondary)' }}>T{entity.teamId}</span>
        <span style={{ color: 'var(--text-muted)' }}>·</span>
        <span style={{ color: 'var(--text-secondary)' }}>{entity.type}</span>
        <span style={{ flex: 1 }} />
        <span style={{
          fontSize: 9, padding: '1px 6px', borderRadius: 2,
          background: entity.active && !captured ? 'rgba(0,200,120,0.2)' : 'rgba(255,80,80,0.2)',
          color: entity.active && !captured ? 'var(--accent-ok)' : 'var(--accent-danger)',
          letterSpacing: '0.05em',
        }}>
          {entity.active && !captured ? 'ALIVE' : 'DEAD'}
        </span>
      </div>
      <Stats entries={[
        ['Lat', `${geo.lat.toFixed(6)}°`],
        ['Lon', `${geo.lon.toFixed(6)}°`],
        ['Alt', `${geo.alt.toFixed(1)} m`],
        ['Z (ENU)', `${entity.position.z.toFixed(1)} m`],
        ['Speed', `${sp.toFixed(1)} m/s · ${(sp * 3.6).toFixed(0)} km/h`],
        ['Heading', heading == null ? '—' : `${heading.toFixed(0)}°`],
        ['Climb', `${climb >= 0 ? '+' : ''}${climb.toFixed(1)} m/s`],
        ['Roll', `${rad2deg(euler.roll).toFixed(0)}°`],
        ['Pitch', `${rad2deg(euler.pitch).toFixed(0)}°`],
        ['Yaw', `${rad2deg(euler.yaw).toFixed(0)}°`],
      ]} />
      {showTargetButton && (
        <div style={{ padding: '12px 14px', borderTop: '1px solid var(--border-default)' }}>
          {assignedTargetId === entity.id ? (
            <button
              type="button"
              onClick={async () => {
                const r = await clearTarget(predatorEntityId!);
                if (r.ok) onAssignedTargetIdChange?.(null);
                else setCommandError(r.error ?? 'unknown error');
              }}
              style={{
                width: '100%', padding: '8px 12px', cursor: 'pointer',
                background: 'var(--accent)', color: 'var(--bg-base)',
                border: 'none', fontFamily: 'var(--font-mono)', fontSize: '12px',
                fontWeight: 600, textTransform: 'uppercase', letterSpacing: '0.05em',
              }}>
              Clear predator target
            </button>
          ) : (
            <button
              type="button"
              onClick={async () => {
                const id = entity.id;
                const r = await assignTarget(predatorEntityId!, id);
                if (r.ok) onAssignedTargetIdChange?.(id);
                else setCommandError(r.error ?? 'unknown error');
              }}
              style={{
                width: '100%', padding: '8px 12px', cursor: 'pointer',
                background: 'transparent', color: 'var(--accent)',
                border: '1px solid var(--accent)', fontFamily: 'var(--font-mono)',
                fontSize: '12px', fontWeight: 600,
                textTransform: 'uppercase', letterSpacing: '0.05em',
              }}>
              Set as predator target
            </button>
          )}
          {commandError && (
            <div style={{
              marginTop: '6px',
              color: 'var(--accent-danger)',
              fontFamily: 'var(--font-mono)',
              fontSize: '11px',
            }}>{commandError}</div>
          )}
        </div>
      )}
    </div>
    </>
  );
}

function rad2deg(r: number) { return (r * 180) / Math.PI; }

function Stats({ entries }: { entries: Array<[string, string]> }) {
  return (
    <div style={{ padding: '6px 10px', display: 'grid', gridTemplateColumns: 'auto 1fr', columnGap: 8, rowGap: 2 }}>
      {entries.map(([k, v]) => (
        <div key={k} style={{ display: 'contents' }}>
          <span style={{ color: 'var(--text-muted)' }}>{k}</span>
          <span style={{ color: 'var(--text-primary)', textAlign: 'right' }}>{v}</span>
        </div>
      ))}
    </div>
  );
}
