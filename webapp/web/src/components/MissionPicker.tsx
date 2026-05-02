import { useEffect, useState } from 'react';
import type { MissionStartResponse } from '../types';
import { listMissions, startMission, stopMission, pauseMission, resumeMission, getStatus } from '../lib/api';

export interface MissionPickerProps {
  onStarted: (resp: MissionStartResponse) => void;
  onStopped: () => void;
}

// SCRIMMAGE's time_warp = simulation seconds per real second.
// 1 = real time, 10 = predator_prey_boids' built-in default, <1 = slow motion.
const SPEED_PRESETS = [0.5, 1, 5, 10, 20, 50];
const DEFAULT_SPEED = 10;
const DEFAULT_MISSION = 'predator_prey_boids.xml';

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
        const preferred = list.find(m => m === DEFAULT_MISSION) ?? list[0] ?? '';
        setSelected(preferred);
      })
      .catch(e => setError(String(e)));
  }, []);

  // Detect mission auto-completion: scrimmage exits naturally when its end_condition
  // triggers. Poll /api/status every 2s while we think a mission is running, and
  // run the same handleStopped flow if the launcher reports idle.
  useEffect(() => {
    if (running === null) return; // only poll while a mission is supposedly active
    let alive = true;
    const tick = async () => {
      try {
        const s = await getStatus();
        if (!alive) return;
        if (s.status === 'idle') {
          setRunning(null);
          setPaused(false);
          onStopped();
        } else if (typeof s.paused === 'boolean' && s.paused !== paused) {
          // Sync paused state in case it changed externally (e.g. via curl).
          setPaused(s.paused);
        }
      } catch {
        // Network blip; try again next tick.
      }
    };
    const interval = setInterval(tick, 2000);
    return () => { alive = false; clearInterval(interval); };
  }, [running, paused, onStopped]);

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
    <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
      <label>Mission</label>
      <select
        value={selected}
        onChange={e => setSelected(e.target.value)}
        disabled={busy || running !== null}
        style={{ minWidth: 180 }}
      >
        {missions.map(m => <option key={m} value={m}>{m}</option>)}
      </select>

      <label style={{ marginLeft: 8 }}>Speed</label>
      <select
        value={timeWarp}
        onChange={e => setTimeWarp(Number(e.target.value))}
        disabled={busy}
        title="Simulation speed (time warp). Takes effect on next Start."
      >
        {SPEED_PRESETS.map(s => (
          <option key={s} value={s}>{s}x{s === 1 ? ' (real)' : ''}</option>
        ))}
      </select>

      <button onClick={handleStart} disabled={busy || running !== null || !selected}>▶ Start</button>
      <button onClick={handlePauseResume} disabled={busy || running === null}>
        {paused ? '▶ Resume' : '❚❚ Pause'}
      </button>
      <button onClick={handleStop} disabled={busy || running === null}>■ Stop</button>

      <span style={{
        marginLeft: 8,
        fontSize: 11, fontWeight: 600, textTransform: 'uppercase', letterSpacing: '0.15em',
        color: paused ? 'var(--accent-warn)' : running ? 'var(--accent-ok)' : 'var(--text-muted)',
      }}>
        {running
          ? `${paused ? '◐ Paused' : '● Live'} · ${running} @ ${timeWarp}x`
          : '○ Idle'}
      </span>
      {error && (
        <span style={{ marginLeft: 8, color: 'var(--accent-danger)', fontSize: 11 }}>
          {error}
        </span>
      )}
    </div>
  );
}
