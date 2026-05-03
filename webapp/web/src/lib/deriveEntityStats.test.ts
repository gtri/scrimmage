import { describe, expect, it } from 'vitest';
import {
  speed,
  headingDeg,
  climbRateMps,
  eulerFromQuat,
  enuToGeo,
} from './deriveEntityStats';

describe('speed', () => {
  it('zero vector → 0', () => {
    expect(speed({ x: 0, y: 0, z: 0 })).toBe(0);
  });
  it('(3, 4, 0) → 5', () => {
    expect(speed({ x: 3, y: 4, z: 0 })).toBeCloseTo(5);
  });
  it('vertical-only velocity counts toward speed', () => {
    expect(speed({ x: 0, y: 0, z: 5 })).toBeCloseTo(5);
  });
});

describe('headingDeg', () => {
  it('pure north → 0', () => {
    expect(headingDeg({ x: 0, y: 1, z: 0 })).toBeCloseTo(0);
  });
  it('pure east → 90', () => {
    expect(headingDeg({ x: 1, y: 0, z: 0 })).toBeCloseTo(90);
  });
  it('pure south → 180', () => {
    expect(headingDeg({ x: 0, y: -1, z: 0 })).toBeCloseTo(180);
  });
  it('pure west → 270', () => {
    expect(headingDeg({ x: -1, y: 0, z: 0 })).toBeCloseTo(270);
  });
  it('vertical-only velocity → null', () => {
    expect(headingDeg({ x: 0, y: 0, z: 5 })).toBeNull();
  });
  it('near-zero horizontal velocity → null', () => {
    expect(headingDeg({ x: 0.05, y: 0.05, z: 5 })).toBeNull();
  });
});

describe('climbRateMps', () => {
  it('returns z component', () => {
    expect(climbRateMps({ x: 1, y: 2, z: 3 })).toBe(3);
    expect(climbRateMps({ x: 0, y: 0, z: -2.5 })).toBe(-2.5);
  });
});

describe('eulerFromQuat', () => {
  it('identity quat → 0/0/0', () => {
    const e = eulerFromQuat({ w: 1, x: 0, y: 0, z: 0 });
    expect(e.roll).toBeCloseTo(0);
    expect(e.pitch).toBeCloseTo(0);
    expect(e.yaw).toBeCloseTo(0);
  });
  it('90° yaw quat → yaw ≈ π/2', () => {
    const s = Math.sin(Math.PI / 4);
    const c = Math.cos(Math.PI / 4);
    const e = eulerFromQuat({ w: c, x: 0, y: 0, z: s });
    expect(e.roll).toBeCloseTo(0);
    expect(e.pitch).toBeCloseTo(0);
    expect(e.yaw).toBeCloseTo(Math.PI / 2);
  });
});

describe('enuToGeo', () => {
  const origin = { lat: 35.721025, lon: -120.767925, alt: 300 };
  it('zero offset returns the origin (within tolerance)', () => {
    const g = enuToGeo(origin, 0, 0, 0);
    expect(g.lat).toBeCloseTo(origin.lat, 5);
    expect(g.lon).toBeCloseTo(origin.lon, 5);
    expect(g.alt).toBeCloseTo(origin.alt, 0);
  });
  it('1000 m north shifts latitude by ~1000/111320 deg', () => {
    const g = enuToGeo(origin, 0, 1000, 0);
    expect(g.lat).toBeCloseTo(origin.lat + 1000 / 111320, 4);
  });
});
