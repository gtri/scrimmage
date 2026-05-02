import { useEffect, useState } from 'react';
import type { MissionStartResponse } from '../types';
import { listMissions, startMission, stopMission } from '../lib/api';

export interface MissionPickerProps {
  onStarted: (resp: MissionStartResponse) => void;
  onStopped: () => void;
}

export function MissionPicker({ onStarted, onStopped }: MissionPickerProps) {
  const [missions, setMissions] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>('');
  const [running, setRunning] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    listMissions()
      .then(list => {
        setMissions(list);
        const preferred = list.find(m => m === 'capture-the-flag.xml') ?? list[0] ?? '';
        setSelected(preferred);
      })
      .catch(e => setError(String(e)));
  }, []);

  async function handleStart() {
    if (!selected) return;
    setBusy(true); setError(null);
    try {
      const resp = await startMission(selected);
      setRunning(selected);
      onStarted(resp);
    } catch (e) {
      setError(String(e));
    } finally { setBusy(false); }
  }

  async function handleStop() {
    setBusy(true); setError(null);
    try {
      await stopMission();
      setRunning(null);
      onStopped();
    } catch (e) {
      setError(String(e));
    } finally { setBusy(false); }
  }

  return (
    <div style={{ display: 'flex', gap: 8, alignItems: 'center', padding: 8, background: '#1a1a1a', color: '#eee' }}>
      <label>Mission:</label>
      <select
        value={selected}
        onChange={e => setSelected(e.target.value)}
        disabled={busy || running !== null}
        style={{ padding: 4 }}
      >
        {missions.map(m => <option key={m} value={m}>{m}</option>)}
      </select>
      <button onClick={handleStart} disabled={busy || running !== null || !selected}>Start</button>
      <button onClick={handleStop} disabled={busy || running === null}>Stop</button>
      <span style={{ marginLeft: 16, color: running ? '#7f7' : '#999' }}>
        {running ? `Running: ${running}` : 'Idle'}
      </span>
      {error && <span style={{ marginLeft: 16, color: '#f77' }}>{error}</span>}
    </div>
  );
}
