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
          width: 28, height: 28, borderRadius: '50%', border: '1px solid #555',
          background: 'rgba(20,20,20,0.85)', color: '#ccc', cursor: 'pointer',
          fontSize: 14, fontWeight: 'bold',
        }}
      >?</button>
    );
  }

  return (
    <div style={{
      position: 'absolute', top: 8, right: 8, zIndex: 10,
      width: 240, padding: 10,
      background: 'rgba(20,20,20,0.92)', color: '#ddd',
      border: '1px solid #555', borderRadius: 6,
      fontSize: 12, lineHeight: 1.5,
    }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
        <strong style={{ color: '#fff' }}>Camera controls</strong>
        <button
          onClick={toggle}
          title="Hide"
          style={{
            background: 'transparent', border: 'none', color: '#888',
            cursor: 'pointer', fontSize: 16, padding: 0, lineHeight: 1,
          }}
        >×</button>
      </div>

      <div style={{ marginBottom: 6, color: '#888' }}>Mouse</div>
      <ControlRow keys="Left-drag" action="Pan" />
      <ControlRow keys="Right-drag" action="Tilt / rotate" />
      <ControlRow keys="Wheel" action="Zoom" />

      <div style={{ marginTop: 8, marginBottom: 6, color: '#888' }}>Trackpad</div>
      <ControlRow keys="One-finger drag" action="Pan" />
      <ControlRow keys="Pinch" action="Zoom" />
      <ControlRow keys="Ctrl + drag" action="Tilt / rotate" />

      <div style={{ marginTop: 10, paddingTop: 8, borderTop: '1px solid #333', color: '#888', fontSize: 11 }}>
        Use the <strong style={{ color: '#ccc' }}>Recenter</strong> button to refit drones in view.
      </div>
    </div>
  );
}

function ControlRow({ keys, action }: { keys: string; action: string }) {
  return (
    <div style={{ display: 'flex', justifyContent: 'space-between', padding: '1px 0' }}>
      <span style={{ color: '#aaa' }}>{keys}</span>
      <span style={{ color: '#ddd' }}>{action}</span>
    </div>
  );
}
