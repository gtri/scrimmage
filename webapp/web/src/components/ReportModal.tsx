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
  const firstDivider = lines.findIndex(l => /^=+$/.test(l.trim()));
  const reportLines = firstDivider >= 0 ? lines.slice(firstDivider) : lines;

  return (
    <div
      onClick={onClose}
      style={{
        position: 'fixed', inset: 0,
        background: 'rgba(7, 16, 26, 0.78)',
        backdropFilter: 'blur(3px)',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        zIndex: 100,
      }}
    >
      <div
        onClick={e => e.stopPropagation()}
        style={{
          width: 'min(760px, 92vw)', maxHeight: '85vh',
          background: 'linear-gradient(180deg, #14222e 0%, #0a141c 100%)',
          color: 'var(--text-primary)',
          border: '1px solid var(--border-strong)',
          borderRadius: 2,
          boxShadow: '0 12px 40px rgba(0,0,0,0.6), 0 0 0 1px var(--accent-glow)',
          display: 'flex', flexDirection: 'column',
        }}
      >
        <div style={{
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          padding: '12px 18px',
          borderBottom: '1px solid var(--border-default)',
          background: 'linear-gradient(180deg, rgba(95,211,232,0.08), transparent)',
        }}>
          <div style={{
            display: 'flex', alignItems: 'baseline', gap: 10,
          }}>
            <span style={{
              fontSize: 14, fontWeight: 700,
              letterSpacing: '0.2em', textTransform: 'uppercase',
              color: 'var(--accent)',
            }}>Mission Report</span>
            <span style={{
              fontSize: 10, letterSpacing: '0.2em', textTransform: 'uppercase',
              color: 'var(--text-muted)',
            }}>Post-action summary</span>
          </div>
          <button
            onClick={onClose}
            title="Close"
            style={{
              background: 'transparent', border: 'none', padding: 0,
              color: 'var(--text-secondary)', fontSize: 22, lineHeight: 1,
              letterSpacing: 0, textTransform: 'none', boxShadow: 'none',
            }}
          >×</button>
        </div>

        <pre style={{
          margin: 0, padding: '16px 18px',
          fontFamily: 'var(--font-mono)',
          fontSize: 12, lineHeight: 1.5,
          color: '#cfdbe2',
          background: 'var(--bg-deepest)',
          overflow: 'auto', flex: 1,
          whiteSpace: 'pre',
        }}>{reportLines.join('')}</pre>

        <div style={{
          padding: '8px 18px',
          borderTop: '1px solid var(--border-default)',
          color: 'var(--text-muted)',
          fontSize: 10, letterSpacing: '0.15em', textTransform: 'uppercase',
        }}>
          Captured from scrimmage stdout · click outside or × to dismiss
        </div>
      </div>
    </div>
  );
}
