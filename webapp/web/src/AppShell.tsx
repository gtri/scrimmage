import { CesiumViewer } from './components/CesiumViewer';

export function AppShell() {
  return (
    <div style={{ position: 'absolute', inset: 0 }}>
      <CesiumViewer />
    </div>
  );
}
