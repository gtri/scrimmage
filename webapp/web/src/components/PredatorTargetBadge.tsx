import type { PredatorTargetSummary } from '../lib/predatorTarget';

export interface PredatorTargetBadgeProps {
  target: PredatorTargetSummary | null;
  onClick: (targetId: number) => void;
}

export function PredatorTargetBadge({ target, onClick }: PredatorTargetBadgeProps) {
  if (!target) return null;
  const distance = target.distanceM < 1000
    ? `${target.distanceM.toFixed(1)} m`
    : `${(target.distanceM / 1000).toFixed(2)} km`;
  return (
    <button
      onClick={() => onClick(target.targetId)}
      title="Fly camera to the predator's current target and follow it"
      style={{
        position: 'absolute', top: 8, left: 120, zIndex: 10,
        display: 'inline-flex', alignItems: 'center', gap: 6,
        padding: '4px 10px',
        background: 'var(--bg-panel)',
        border: '1px solid var(--border-default)',
        borderLeft: '3px solid var(--team-red)',
        borderRadius: 2,
        color: 'var(--text-primary)',
        fontFamily: 'var(--font-mono)', fontSize: 11,
        cursor: 'pointer',
        boxShadow: '0 2px 6px rgba(0,0,0,0.3)',
      }}
    >
      <span style={{ color: 'var(--team-red)' }}>🎯</span>
      <span style={{ color: 'var(--team-red)', fontWeight: 700 }}>#{target.predatorId}</span>
      <span style={{ color: 'var(--text-muted)' }}>→</span>
      <span style={{ color: 'var(--team-blue)', fontWeight: 700 }}>#{target.targetId}</span>
      <span style={{ color: 'var(--text-muted)' }}>·</span>
      <span style={{ color: 'var(--text-secondary)' }}>{distance}</span>
    </button>
  );
}
