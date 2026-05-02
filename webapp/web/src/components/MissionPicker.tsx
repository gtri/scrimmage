import { useEffect, useState } from 'react';
import type { MissionStartResponse } from '../types';
import { listMissions, startMission, stopMission, pauseMission, resumeMission } from '../lib/api';

export interface MissionPickerProps {
  onStarted: (resp: MissionStartResponse) => void;
  onStopped: () => void;
}

// SCRIMMAGE's time_warp = simulation seconds per real second.
// 1 = real time, 20 = capture-the-flag's default (fast), <1 = slow motion.
const SPEED_PRESETS = [0.5, 1, 5, 10, 20, 50];
const DEFAULT_SPEED = 20;

export function MissionPicker({ onStarted, onStopped }: MissionPickerProps) {
  const [missions, setMissions] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>('');
  const [timeWarp, setTimeWarp] = useState<number>(DEFAULT_SPEED);
  const [running, setRunning] = useState<string | null>(null);
  const [paused, setPaused] = useState(false);
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
      const resp = await startMission(selected, timeWarp);
      setRunning(selected);
      setPaused(false);
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
      setPaused(false);
      onStopped();
    } catch (e) {
      setError(String(e));
    } finally { setBusy(false); }
  }

  async function handlePauseResume() {
    setBusy(true); setError(null);
    try {
      if (paused) { await resumeMission(); setPaused(false); }
      else        { await pauseMission();  setPaused(true);  }
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
      <label style={{ marginLeft: 8 }}>Speed:</label>
      <select
        value={timeWarp}
        onChange={e => setTimeWarp(Number(e.target.value))}
        disabled={busy}
        title="Simulation speed (time warp). Takes effect on next Start."
        style={{ padding: 4 }}
      >
        {SPEED_PRESETS.map(s => (
          <option key={s} value={s}>{s}x{s === 1 ? ' (real time)' : ''}</option>
        ))}
      </select>
      <button onClick={handleStart} disabled={busy || running !== null || !selected}>Start</button>
      <button onClick={handlePauseResume} disabled={busy || running === null}>
        {paused ? 'Resume' : 'Pause'}
      </button>
      <button onClick={handleStop} disabled={busy || running === null}>Stop</button>
      <span style={{ marginLeft: 16, color: paused ? '#fc7' : running ? '#7f7' : '#999' }}>
        {running ? `${paused ? 'Paused' : 'Running'}: ${running} @ ${timeWarp}x` : 'Idle'}
      </span>
      {error && <span style={{ marginLeft: 16, color: '#f77' }}>{error}</span>}
    </div>
  );
}
