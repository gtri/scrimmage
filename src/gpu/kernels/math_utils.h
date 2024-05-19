#ifndef KERNEL_MATH_UTILS_H
#define KERNEL_MATH_UTILS_H

#include <common.h>

// These should only be used for compile-time constants. The OpenCL implementation
// provides similar functions
#define RADIANS(degrees) (M_PI * (degrees) / 180);
#define DEGREES(radians) (180 * M_1_PI*(radians));

typedef fp4_t quat_t;
// Quanternion-Euler Conversions
fp_t quat_roll(fp4_t q) {
    return atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
}

fp_t quat_pitch(fp4_t q) {
  return asin(2 * (q.w * q.y - q.z * q.x));
}

fp_t quat_yaw(fp4_t q) {
  return atan2(2*(q.w * q.z + q.x * q.y), 1 - 2*(q.y * q.y + q.z * q.z));
}

fp4_t quat_from_euler(fp_t roll, fp_t pitch, fp_t yaw) {
  fp4_t q;

  fp_t sr = sin(roll / 2);
  fp_t cr = cos(roll / 2);
  fp_t sy = sin(yaw / 2);
  fp_t cy = cos(yaw / 2);
  fp_t sp = sin(pitch / 2);
  fp_t cp = cos(pitch / 2);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

// These are macros to enable functions to pass in their own motion models.
// OpenCL 1.1 does not permit the use of function pointers.
#define RK4(x, u, t, dt, model) {\
    fp8_t k1, k2, k3, k4; \
    k1 = (model((x), (u), (t))); \
    k2 = (model((x) + (dt)*k1 / 2, (u), (t) + (dt) / 2)); \
    k3 = (model((x) + (dt)*k2 / 2, (u), (t) + (dt) / 2)); \
    k4 = (model((x) + (dt)*k3, (u), (t) + (dt))); \
    x += ((dt)*(k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6)); \
    }

#define FORWARD_EULER(x, y, t, dt, model) (x = x + dt*model(x, u, t))

#endif //KERNEL_MATH_UTILS_H
