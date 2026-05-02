import { describe, expect, it } from 'vitest';
import * as Cesium from 'cesium';
import { enuToCartesian } from './enuToCartesian';

describe('enuToCartesian', () => {
  const origin = { lat: 35.721025, lon: -120.767925, alt: 300 };

  it('returns the origin position when local offset is (0,0,0)', () => {
    const result = enuToCartesian(origin, 0, 0, 0);
    const expected = Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt);
    expect(result.x).toBeCloseTo(expected.x, 1);
    expect(result.y).toBeCloseTo(expected.y, 1);
    expect(result.z).toBeCloseTo(expected.z, 1);
  });

  it('moves north (positive y) by ~111 km per degree of latitude', () => {
    // 1 degree latitude ≈ 111,320 m. We move 1000 m north and check the latitude delta.
    const result = enuToCartesian(origin, 0, 1000, 0);
    const carto = Cesium.Cartographic.fromCartesian(result);
    const latDeg = Cesium.Math.toDegrees(carto.latitude);
    const expectedLatDeg = origin.lat + 1000 / 111320;
    expect(latDeg).toBeCloseTo(expectedLatDeg, 4);
  });

  it('moves east (positive x) and stays close to origin altitude', () => {
    const result = enuToCartesian(origin, 1000, 0, 0);
    const carto = Cesium.Cartographic.fromCartesian(result);
    expect(carto.height).toBeCloseTo(origin.alt, 0); // within 1 m
  });
});
