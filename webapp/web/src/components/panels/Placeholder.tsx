export interface PlaceholderProps { title: string; description: string; }

export function Placeholder({ title, description }: PlaceholderProps) {
  return (
    <div style={{ padding: 12, borderBottom: '1px solid #333', color: '#aaa' }}>
      <div style={{ fontWeight: 'bold', color: '#ccc', marginBottom: 4 }}>{title}</div>
      <div style={{ fontSize: 12 }}>{description}</div>
    </div>
  );
}
