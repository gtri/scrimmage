import { describe, expect, it } from 'vitest';
import { predatorTarget } from './predatorTarget';
import type { EntityDto, FrameDto } from '../types';

function entity(
  id: number,
  teamId: number,
  x: number,
  y: number,
  z: number,
): EntityDto {
  return {
    id, teamId, subSwarmId: 0, type: 'test', active: true,
    position: { x, y, z },
    velocity: { x: 0, y: 0, z: 0 },
    orientation: { w: 1, x: 0, y: 0, z: 0 },
  };
}

function frame(...entities: EntityDto[]): FrameDto {
  return { time: 0, entities };
}

describe('predatorTarget', () => {
  it('empty frame → null', () => {
    expect(predatorTarget(frame())).toBeNull();
  });

  it('prey only (no predator) → null', () => {
    expect(predatorTarget(frame(
      entity(1, 1, 0, 0, 0),
      entity(2, 1, 100, 0, 0),
    ))).toBeNull();
  });

  it('predator only (no prey) → null', () => {
    expect(predatorTarget(frame(
      entity(99, 2, 0, 0, 0),
    ))).toBeNull();
  });

  it('predator + same-team teammates only → null', () => {
    expect(predatorTarget(frame(
      entity(99, 2, 0, 0, 0),
      entity(98, 2, 50, 0, 0),
    ))).toBeNull();
  });

  it('predator + 3 prey → picks the closest, distance is correct', () => {
    const result = predatorTarget(frame(
      entity(99, 2, 0, 0, 0),
      entity(1, 1, 100, 0, 0),     // dist 100
      entity(2, 1, 30, 40, 0),     // dist 50
      entity(3, 1, 200, 0, 0),     // dist 200
    ));
    expect(result).not.toBeNull();
    expect(result!.predatorId).toBe(99);
    expect(result!.targetId).toBe(2);
    expect(result!.distanceM).toBeCloseTo(50);
  });

  it('multiple predators → returns first one only (documented limitation)', () => {
    const result = predatorTarget(frame(
      entity(99, 2, 0, 0, 0),
      entity(100, 2, 1000, 0, 0),
      entity(1, 1, 50, 0, 0),
    ));
    expect(result!.predatorId).toBe(99);
    expect(result!.targetId).toBe(1);
  });
});
