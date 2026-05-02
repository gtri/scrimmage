import type { MissionStartResponse } from '../types';

const BASE = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

export async function listMissions(): Promise<string[]> {
  const r = await fetch(`${BASE}/api/missions`);
  if (!r.ok) throw new Error(`listMissions: ${r.status}`);
  return r.json();
}

export async function startMission(name: string, timeWarp?: number): Promise<MissionStartResponse> {
  const r = await fetch(`${BASE}/api/missions/start`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, timeWarp }),
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

export async function pauseMission(): Promise<void> {
  const r = await fetch(`${BASE}/api/missions/pause`, { method: 'POST' });
  if (!r.ok) throw new Error(`pauseMission: ${r.status} ${await r.text()}`);
}

export async function resumeMission(): Promise<void> {
  const r = await fetch(`${BASE}/api/missions/resume`, { method: 'POST' });
  if (!r.ok) throw new Error(`resumeMission: ${r.status} ${await r.text()}`);
}

export async function fetchReport(): Promise<string[]> {
  const r = await fetch(`${BASE}/api/missions/report`);
  if (!r.ok) throw new Error(`fetchReport: ${r.status}`);
  const data: { lines: string[] } = await r.json();
  return data.lines ?? [];
}

export interface StatusInfo {
  status: 'idle' | 'running' | 'paused';
  mission?: string;
  paused?: boolean;
  uptime_s?: number;
}

export async function getStatus(): Promise<StatusInfo> {
  const r = await fetch(`${BASE}/api/status`);
  if (!r.ok) throw new Error(`getStatus: ${r.status}`);
  return r.json();
}
