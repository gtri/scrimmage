import type { FrameDto } from '../types';

export interface EntityListProps {
  frame: FrameDto | null;
  selectedId: number | null;
  onSelect: (id: number | null) => void;
}

export function EntityList({ frame, selectedId, onSelect }: EntityListProps) {
  if (!frame) {
    return (
      <div style={{
        padding: 16, color: 'var(--text-muted)', fontSize: 12,
        fontStyle: 'italic', textAlign: 'center',
      }}>
        Awaiting frames…
      </div>
    );
  }
  const sorted = [...frame.entities].sort((a, b) => a.teamId - b.teamId || a.id - b.id);
  return (
    <div style={{ padding: '8px 6px' }}>
      <div style={{
        marginBottom: 8, padding: '0 6px',
        fontFamily: 'var(--font-mono)', fontSize: 10,
        color: 'var(--text-muted)', letterSpacing: '0.08em',
      }}>
        {frame.entities.length} ENTITIES · t={frame.time.toFixed(1)}s
      </div>
      {sorted.map(e => {
        const isSelected = e.id === selectedId;
        const teamColor =
          e.teamId === 1 ? 'var(--team-blue)' :
          e.teamId === 2 ? 'var(--team-red)' :
          'var(--text-secondary)';
        return (
          <button
            key={e.id}
            onClick={() => onSelect(isSelected ? null : e.id)}
            title={`Click to ${isSelected ? 'deselect' : 'select'} entity #${e.id}`}
            style={{
              display: 'flex', alignItems: 'center', gap: 8,
              width: '100%', textAlign: 'left',
              padding: '6px 10px', marginBottom: 2,
              border: `1px solid ${isSelected ? 'var(--border-accent)' : 'transparent'}`,
              borderLeft: `3px solid ${teamColor}`,
              borderRadius: 2,
              background: isSelected
                ? 'linear-gradient(90deg, rgba(95,211,232,0.18), rgba(95,211,232,0.04))'
                : 'transparent',
              boxShadow: isSelected ? '0 0 0 1px var(--accent-glow)' : 'none',
              color: 'var(--text-primary)',
              opacity: e.active ? 1 : 0.4,
              cursor: 'pointer',
              fontFamily: 'var(--font-mono)', fontSize: 12,
              textTransform: 'none', letterSpacing: 0,
              fontWeight: 400,
              transition: 'background 0.1s, border-color 0.1s',
            }}
          >
            <span style={{ color: teamColor, fontWeight: 600 }}>#{e.id}</span>
            <span style={{ color: 'var(--text-muted)' }}>·</span>
            <span style={{ color: 'var(--text-secondary)' }}>T{e.teamId}</span>
            <span style={{ color: 'var(--text-muted)' }}>·</span>
            <span style={{ color: 'var(--text-secondary)', fontSize: 11 }}>{e.type}</span>
          </button>
        );
      })}
    </div>
  );
}
