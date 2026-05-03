import type { FrameDto } from '../types';

const PREY_TEAM_ID = 1;
const PREDATOR_TEAM_ID = 2;

export interface PredatorTargetSummary {
  predatorId: number;
  targetId: number;
  distanceM: number;
}

/**
 * Identify the predator's current target.
 *
 * If `assignedTargetId` is provided AND the entity is present in the frame,
 * uses it directly (matches an operator-issued TargetAssignment that the
 * sim-side RemotePredator has hard-locked onto). Otherwise falls back to the
 * nearest-team-1 heuristic the upstream Predator uses, which is also what
 * RemotePredator falls through to when no assignment is active.
 *
 * Limitations:
 * - With multiple team-2 entities in a frame, only the first is considered.
 *   Multi-predator handling is intentionally deferred.
 * - Returns null when there's no predator OR no prey to chase.
 */
export function predatorTarget(
  frame: FrameDto,
  assignedTargetId?: number | null,
): PredatorTargetSummary | null {
  const predator = frame.entities.find(e => e.teamId === PREDATOR_TEAM_ID);
  if (!predator) return null;

  // Operator-assigned target wins, if it's still in the frame.
  if (assignedTargetId != null) {
    const assigned = frame.entities.find(e => e.id === assignedTargetId);
    if (assigned) {
      const dx = assigned.position.x - predator.position.x;
      const dy = assigned.position.y - predator.position.y;
      const dz = assigned.position.z - predator.position.z;
      return {
        predatorId: predator.id,
        targetId: assigned.id,
        distanceM: Math.sqrt(dx * dx + dy * dy + dz * dz),
      };
    }
    // Assigned entity left the frame (captured); fall through to heuristic.
  }

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
