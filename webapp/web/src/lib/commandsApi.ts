const API_URL = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

export interface CommandResult {
  ok: boolean;
  error?: string;
}

async function postJson(path: string, body: unknown): Promise<CommandResult> {
  const res = await fetch(`${API_URL}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  let parsed: CommandResult | null = null;
  try { parsed = await res.json() as CommandResult; } catch { /* no-op */ }
  if (!res.ok) {
    return { ok: false, error: parsed?.error ?? `HTTP ${res.status}` };
  }
  return parsed ?? { ok: false, error: 'empty response' };
}

export function assignTarget(predatorId: number, targetId: number): Promise<CommandResult> {
  return postJson('/api/commands/target-assignment', { predatorId, targetId });
}

export function clearTarget(predatorId: number): Promise<CommandResult> {
  return postJson('/api/commands/target-assignment', { predatorId, targetId: 0 });
}
