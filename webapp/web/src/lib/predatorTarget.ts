import type { FrameDto } from '../types';

const PREY_TEAM_ID = 1;
const PREDATOR_TEAM_ID = 2;

export interface PredatorTargetSummary {
  predatorId: number;
  targetId: number;
  distanceM: number;
}

/**
 * Identify the predator's current target by the same heuristic the Predator
 * autonomy plugin uses internally: nearest team-1 entity by 3D distance.
 *
 * Limitations:
 * - With multiple team-2 entities in a frame, only the first is considered.
 *   Multi-predator handling is intentionally deferred.
 * - Returns null when there's no predator OR no prey to chase.
 */
export function predatorTarget(frame: FrameDto): PredatorTargetSummary | null {
  const predator = frame.entities.find(e => e.teamId === PREDATOR_TEAM_ID);
  if (!predator) return null;

  let bestId: number | null = null;
  let bestSq = Infinity;
  for (const e of frame.entities) {
    if (e.teamId !== PREY_TEAM_ID) continue;
    const dx = e.position.x - predator.position.x;
    const dy = e.position.y - predator.position.y;
    const dz = e.position.z - predator.position.z;
    const sq = dx * dx + dy * dy + dz * dz;
    if (sq < bestSq) { bestSq = sq; bestId = e.id; }
  }
  if (bestId == null) return null;

  return {
    predatorId: predator.id,
    targetId: bestId,
    distanceM: Math.sqrt(bestSq),
  };
}
