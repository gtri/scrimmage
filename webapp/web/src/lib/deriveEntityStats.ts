import * as Cesium from 'cesium';
import type { Quat, Vec3 } from '../types';
import { enuToCartesian, type Origin } from './enuToCartesian';

const HORIZONTAL_DEADBAND_MPS = 0.1;

export function speed(v: Vec3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Heading in degrees clockwise from north, ENU convention (x=east, y=north).
 * Returns null when horizontal speed is below the deadband — heading is
 * undefined for a drone that's only climbing or hovering.
 */
export function headingDeg(v: Vec3): number | null {
  const horiz = Math.sqrt(v.x * v.x + v.y * v.y);
  if (horiz < HORIZONTAL_DEADBAND_MPS) return null;
  const rad = Math.atan2(v.x, v.y);
  const deg = (rad * 180) / Math.PI;
  return (deg + 360) % 360;
}

export function climbRateMps(v: Vec3): number {
  return v.z;
}

/**
 * Convert a (w, x, y, z) quaternion to ZYX intrinsic Euler angles in radians.
 * roll = rotation about x, pitch = rotation about y, yaw = rotation about z.
 */
export function eulerFromQuat(q: Quat): { roll: number; pitch: number; yaw: number } {
  const { w, x, y, z } = q;
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  const sinp = 2 * (w * y - z * x);
  const pitch =
    Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return { roll, pitch, yaw };
}

/**
 * Convert a local ENU offset (meters from origin) to geographic lat/lon/alt.
 * Composes the existing enuToCartesian with Cesium's cartographic conversion.
 */
export function enuToGeo(
  origin: Origin,
  east: number,
  north: number,
  up: number,
): { lat: number; lon: number; alt: number } {
  const cartesian = enuToCartesian(origin, east, north, up);
  const carto = Cesium.Cartographic.fromCartesian(cartesian);
  return {
    lat: Cesium.Math.toDegrees(carto.latitude),
    lon: Cesium.Math.toDegrees(carto.longitude),
    alt: carto.height,
  };
}
