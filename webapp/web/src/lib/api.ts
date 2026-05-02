import type { MissionStartResponse } from '../types';

const BASE = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

export async function listMissions(): Promise<string[]> {
  const r = await fetch(`${BASE}/api/missions`);
  if (!r.ok) throw new Error(`listMissions: ${r.status}`);
  return r.json();
}

export async function startMission(name: string): Promise<MissionStartResponse> {
  const r = await fetch(`${BASE}/api/missions/start`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  });
  if (!r.ok) {
    const err = await r.text();
    throw new Error(`startMission failed: ${r.status} ${err}`);
  }
  return r.json();
}

export async function stopMission(): Promise<void> {
  const r = await fetch(`${BASE}/api/missions/stop`, { method: 'POST' });
  if (!r.ok) throw new Error(`stopMission: ${r.status}`);
}
