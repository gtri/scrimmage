import type { EntityDto } from '../types';
import type { Origin } from '../lib/enuToCartesian';
import {
  speed,
  headingDeg,
  climbRateMps,
  eulerFromQuat,
  enuToGeo,
} from '../lib/deriveEntityStats';
import type { ProjectedPoint } from './CesiumViewer';

export interface EntityInspectorCardProps {
  entity: EntityDto;
  origin: Origin;
  screenPosition: ProjectedPoint | null;
  mode: 'live' | 'captured';
  killerId?: number | null;
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
}: EntityInspectorCardProps) {
  if (!screenPosition || !screenPosition.visible) return null;

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
    <div
      style={{
        position: 'absolute', left, top, width: CARD_WIDTH,
        background: 'var(--bg-panel)',
        border: `1px solid ${captured ? 'var(--accent-danger)' : 'var(--border-default)'}`,
        borderLeft: `3px solid ${teamColor}`,
        boxShadow: '0 4px 12px rgba(0,0,0,0.4)',
        color: 'var(--text-primary)',
        fontFamily: 'var(--font-mono)', fontSize: 11,
        zIndex: 20,
        pointerEvents: 'none',
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
    </div>
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
