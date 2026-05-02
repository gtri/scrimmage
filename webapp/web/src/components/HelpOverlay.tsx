import { useState } from 'react';

const STORAGE_KEY = 'c2.helpOverlay.collapsed';

export function HelpOverlay() {
  const [collapsed, setCollapsed] = useState<boolean>(
    () => localStorage.getItem(STORAGE_KEY) === '1'
  );

  function toggle() {
    const next = !collapsed;
    setCollapsed(next);
    localStorage.setItem(STORAGE_KEY, next ? '1' : '0');
  }

  if (collapsed) {
    return (
      <button
        onClick={toggle}
        title="Show camera controls"
        style={{
          position: 'absolute', top: 8, right: 8, zIndex: 10,
          width: 32, height: 32, padding: 0,
          borderRadius: 2,
        }}
      >?</button>
    );
  }

  return (
    <div style={{
      position: 'absolute', top: 8, right: 8, zIndex: 10,
      width: 260, padding: 0,
      background: 'linear-gradient(180deg, rgba(20,34,46,0.96), rgba(12,24,32,0.96))',
      border: '1px solid var(--border-strong)',
      borderRadius: 2,
      backdropFilter: 'blur(4px)',
      boxShadow: '0 8px 24px rgba(0,0,0,0.5)',
    }}>
      <div style={{
        display: 'flex', justifyContent: 'space-between', alignItems: 'center',
        padding: '8px 12px',
        borderBottom: '1px solid var(--border-default)',
      }}>
        <span style={{
          fontSize: 11, fontWeight: 700, letterSpacing: '0.18em',
          textTransform: 'uppercase', color: 'var(--accent)',
        }}>Camera Controls</span>
        <button
          onClick={toggle}
          title="Hide"
          style={{
            background: 'transparent', border: 'none', padding: 0,
            color: 'var(--text-muted)', fontSize: 18, lineHeight: 1,
            letterSpacing: 0, textTransform: 'none', boxShadow: 'none',
          }}
        >×</button>
      </div>

      <div style={{ padding: 12, fontSize: 12 }}>
        <SectionHeader>Mouse</SectionHeader>
        <ControlRow keys="Left-drag" action="Pan" />
        <ControlRow keys="Right-drag" action="Tilt / rotate" />
        <ControlRow keys="Wheel" action="Zoom" />

        <SectionHeader style={{ marginTop: 10 }}>Trackpad</SectionHeader>
        <ControlRow keys="One-finger drag" action="Pan" />
        <ControlRow keys="Pinch" action="Zoom" />
        <ControlRow keys="Ctrl + drag" action="Tilt / rotate" />

        <div style={{
          marginTop: 12, paddingTop: 10,
          borderTop: '1px solid var(--border-default)',
          color: 'var(--text-muted)', fontSize: 11,
        }}>
          Use <span style={{ color: 'var(--accent)', fontWeight: 600 }}>⊕ Recenter</span> to refit drones in view.
        </div>
      </div>
    </div>
  );
}

function SectionHeader({ children, style }: { children: React.ReactNode; style?: React.CSSProperties }) {
  return (
    <div style={{
      marginBottom: 4,
      fontSize: 9, fontWeight: 700, letterSpacing: '0.22em',
      textTransform: 'uppercase', color: 'var(--text-muted)',
      ...style,
    }}>{children}</div>
  );
}

function ControlRow({ keys, action }: { keys: string; action: string }) {
  return (
    <div style={{
      display: 'flex', justifyContent: 'space-between',
      padding: '2px 0', fontFamily: 'var(--font-mono)', fontSize: 11,
    }}>
      <span style={{ color: 'var(--text-secondary)' }}>{keys}</span>
      <span style={{ color: 'var(--text-primary)' }}>{action}</span>
    </div>
  );
}
