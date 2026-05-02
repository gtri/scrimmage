import type { FrameDto } from '../types';

export interface EntityListProps {
  frame: FrameDto | null;
  selectedId: number | null;
  onSelect: (id: number | null) => void;
}

export function EntityList({ frame, selectedId, onSelect }: EntityListProps) {
  if (!frame) return <div style={{ padding: 12, color: '#666' }}>No frames yet</div>;
  const sorted = [...frame.entities].sort((a, b) => a.teamId - b.teamId || a.id - b.id);
  return (
    <div style={{ padding: 8, fontSize: 13, color: '#ccc' }}>
      <div style={{ marginBottom: 6, color: '#888' }}>
        Entities: {frame.entities.length} (t={frame.time.toFixed(1)})
      </div>
      {sorted.map(e => {
        const isSelected = e.id === selectedId;
        return (
          <button
            key={e.id}
            onClick={() => onSelect(isSelected ? null : e.id)}
            title={`Click to ${isSelected ? 'deselect' : 'select'} entity #${e.id} (also highlights it on the map)`}
            style={{
              display: 'block',
              width: '100%',
              textAlign: 'left',
              padding: '3px 6px',
              marginBottom: 1,
              border: '1px solid transparent',
              borderRadius: 3,
              background: isSelected ? '#2a3a5a' : 'transparent',
              borderColor: isSelected ? '#5a7aaa' : 'transparent',
              color: e.teamId === 1 ? '#7af' : e.teamId === 2 ? '#f77' : '#ccc',
              opacity: e.active ? 1 : 0.4,
              cursor: 'pointer',
              font: 'inherit',
            }}
          >
            #{e.id} · team {e.teamId} · {e.type}
          </button>
        );
      })}
    </div>
  );
}
