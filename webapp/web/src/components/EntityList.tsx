import type { FrameDto } from '../types';

export interface EntityListProps { frame: FrameDto | null; }

export function EntityList({ frame }: EntityListProps) {
  if (!frame) return <div style={{ padding: 12, color: '#666' }}>No frames yet</div>;
  const sorted = [...frame.entities].sort((a, b) => a.teamId - b.teamId || a.id - b.id);
  return (
    <div style={{ padding: 8, fontSize: 13, color: '#ccc' }}>
      <div style={{ marginBottom: 6, color: '#888' }}>
        Entities: {frame.entities.length} (t={frame.time.toFixed(1)})
      </div>
      {sorted.map(e => (
        <div key={e.id} style={{
          padding: '3px 6px',
          color: e.teamId === 1 ? '#7af' : e.teamId === 2 ? '#f77' : '#ccc',
          opacity: e.active ? 1 : 0.4,
        }}>
          #{e.id} · team {e.teamId} · {e.type}
        </div>
      ))}
    </div>
  );
}
