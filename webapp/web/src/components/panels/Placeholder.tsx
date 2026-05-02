export interface PlaceholderProps { title: string; description: string; }

export function Placeholder({ title, description }: PlaceholderProps) {
  return (
    <div style={{
      padding: '12px 14px',
      borderBottom: '1px solid var(--border-default)',
    }}>
      <div style={{
        fontSize: 10, fontWeight: 700, letterSpacing: '0.2em',
        textTransform: 'uppercase', color: 'var(--accent)',
        marginBottom: 6,
        display: 'flex', alignItems: 'center', gap: 6,
      }}>
        <span style={{ width: 8, height: 1, background: 'var(--accent)', display: 'inline-block' }} />
        {title}
      </div>
      <div style={{
        fontSize: 11, lineHeight: 1.5, color: 'var(--text-muted)',
      }}>{description}</div>
    </div>
  );
}
