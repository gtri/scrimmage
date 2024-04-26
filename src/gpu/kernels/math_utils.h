#ifndef KERNEL_MATH_UTILS_H
#define KERNEL_MATH_UTILS_H

// Declares model function used in rk4.
// OpenCL 1.1 does not allow the usage of function pointers,
// so declare this function, and then expect motion models
// to implement their own. 

float quat_yaw(float4 q) {
  return atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(pown(q.y, 2) + pown(q.z, 2)));
}

float quat_roll(float4 q) {
  return atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(pown(q.x, 2) + pown(q.y, 2)));
}

float quat_pitch(float4 q) {
  return asin(2*(q.w*q.y - q.z*q.x));
}

float4 quat_from_euler(float yaw, float pitch, float roll) {
    float4 q;

    float sr = sin(roll / 2);
    float cr = cos(roll / 2);
    float sy = sin(yaw / 2);
    float cy = cos(yaw / 2);
    float sp = sin(pitch / 2);
    float cp = cos(pitch / 2);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// These are macros to enable functions to pass in their own motion models.
// OpenCL 1.1 does not permit the use of function pointers.
#define RK4(x, u, t, dt, model) {\ 
    float8 k1, k2, k3, k4; \
    k1 = model(x, u, t); \
    k2 = model(x + 0.5f*dt*k1, u, t + 0.5f*dt); \
    k3 = model(x + 0.5f*dt*k2, u, t + 0.5f*dt); \
    k4 = model(x + dt*k3, u, t+dt); \
    x += dt*(k1 + 2*k2 + 2*k3 + k4) / 6.0f; \
    }

#define FORWARD_EULER(x, y, t, dt, model) (x = x + dt*model(x, u, t))

#endif //KERNEL_MATH_UTILS_H
