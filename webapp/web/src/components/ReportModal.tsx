export interface ReportModalProps {
  lines: string[];
  onClose: () => void;
}

/**
 * Full-screen modal that surfaces SCRIMMAGE's post-mission report (the metrics
 * summary it prints to stdout on shutdown — SimpleCollisionMetrics, Overall Scores, etc).
 * Renders as preformatted text so the original column alignment + dividers are preserved.
 */
export function ReportModal({ lines, onClose }: ReportModalProps) {
  // Trim leading non-report chatter: start at the first divider line if present.
  // Falls back to showing everything if no divider is found.
  const firstDivider = lines.findIndex(l => /^=+$/.test(l.trim()));
  const reportLines = firstDivider >= 0 ? lines.slice(firstDivider) : lines;

  return (
    <div
      onClick={onClose}
      style={{
        position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.7)',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        zIndex: 100,
      }}
    >
      <div
        onClick={e => e.stopPropagation()}
        style={{
          width: 'min(720px, 90vw)', maxHeight: '85vh',
          background: '#1a1a1a', color: '#ddd',
          border: '1px solid #555', borderRadius: 6,
          display: 'flex', flexDirection: 'column',
        }}
      >
        <div style={{
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          padding: '10px 16px', borderBottom: '1px solid #333',
        }}>
          <strong style={{ color: '#fff' }}>Mission report</strong>
          <button
            onClick={onClose}
            title="Close"
            style={{
              background: 'transparent', border: 'none', color: '#aaa',
              cursor: 'pointer', fontSize: 20, lineHeight: 1, padding: 0,
            }}
          >×</button>
        </div>

        <pre style={{
          margin: 0, padding: 16,
          fontFamily: 'ui-monospace, Menlo, Consolas, monospace',
          fontSize: 12, lineHeight: 1.4, color: '#cfcfcf',
          background: '#111', overflow: 'auto', flex: 1,
          whiteSpace: 'pre',
        }}>{reportLines.join('')}</pre>

        <div style={{
          padding: '8px 16px', borderTop: '1px solid #333',
          color: '#888', fontSize: 11,
        }}>
          Captured from scrimmage stdout on shutdown. Click outside or press × to dismiss.
        </div>
      </div>
    </div>
  );
}
