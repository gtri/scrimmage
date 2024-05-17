#ifndef KERNEL_MATH_UTILS_H
#define KERNEL_MATH_UTILS_H

#include <common.h>

typedef fp4_t quat_t;

// Declares model function used in rk4.
// OpenCL 1.1 does not allow the usage of function pointers,
// so declare this function, and then expect motion models
// to implement their own. 

// Quanternion-Euler Conversions
fp_t quat_yaw(fp4_t q) {
  return atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y * q.y + q.z * q.z));
}

fp_t quat_pitch(fp4_t q) {
  return asin(2 * (q.w * q.y - q.z * q.x));
}

fp_t quat_roll(fp4_t q) {
    return atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x*q.x + q.y*q.y));
}

fp4_t quat_from_euler(fp_t yaw, fp_t  pitch, fp_t roll) {
  fp4_t q;

  fp_t sr = sin(0.5f*roll);
  fp_t cr = cos(0.5f*roll);
  fp_t sy = sin(0.5f*yaw);
  fp_t cy = cos(0.5f*yaw);
  fp_t sp = sin(0.5f*pitch);
  fp_t cp = cos(0.5f*pitch);

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
    k2 = (model((x) + 0.5f*(dt)*k1, (u), (t) + 0.5f*(dt))); \
    k3 = (model((x) + 0.5f*(dt)*k2, (u), (t) + 0.5f*(dt))); \
    k4 = (model((x) + (dt)*k3, (u), (t) + (dt))); \
    x += ((dt)*(k1 + 2.0f*k2 + 2.0f*k3 + k4) / 6.0f); \
    }

#define FORWARD_EULER(x, y, t, dt, model) (x = x + dt*model(x, u, t))

#endif //KERNEL_MATH_UTILS_H
