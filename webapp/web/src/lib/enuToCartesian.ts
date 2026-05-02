import * as Cesium from 'cesium';

export interface Origin { lat: number; lon: number; alt: number; }

/**
 * Convert a local ENU offset (meters) from a geographic origin to a Cartesian3 in ECEF.
 * SCRIMMAGE positions are reported as ENU offsets from the mission's lat/lon/alt origin.
 */
export function enuToCartesian(
  origin: Origin,
  east: number,
  north: number,
  up: number
): Cesium.Cartesian3 {
  const originEcef = Cesium.Cartesian3.fromDegrees(origin.lon, origin.lat, origin.alt);
  const enuFrame = Cesium.Transforms.eastNorthUpToFixedFrame(originEcef);
  const offset = new Cesium.Cartesian3(east, north, up);
  const result = new Cesium.Cartesian3();
  Cesium.Matrix4.multiplyByPoint(enuFrame, offset, result);
  return result;
}
