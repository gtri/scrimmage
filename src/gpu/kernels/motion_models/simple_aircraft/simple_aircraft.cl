#include "math_utils.h"

enum StateParams {
  STATE_X = 0,
  STATE_Y,
  STATE_Z,
  STATE_X_VEL,
  STATE_Y_VEL,
  STATE_Z_VEL,
  STATE_X_ANG_VEL,
  STATE_Y_ANG_VEL,
  STATE_Z_ANG_VEL,
  STATE_QUAT_W,
  STATE_QUAT_X,
  STATE_QUAT_Y,
  STATE_QUAT_Z,
  STATE_NUM_PARAMS
};

enum ModelParams {
    MODEL_X = 0,
    MODEL_Y,
    MODEL_Z,
    MODEL_ROLL,
    MODEL_PITCH,
    MODEL_YAW,
    MODEL_SPEED,
    MODEL_NUM_PARAMS
};

enum ControlParams {
    CONTROL_THRUST = 0,
    CONTROL_ROLL_RATE,
    CONTROL_PITCH_RATE,
    CONTROL_NUM_PARAMS
};

#define MAX_SPEED (40.0f)
#define MIN_SPEED (15.0f)
#define MAX_ROLL (30.0f)
#define MAX_PITCH (30.0f)
#define TURNING_RADIUS (50.0f)
#define SPEED_TARGET (50.0f)
#define RADIUS_SLOPE_PER_SPEED (0.0f)
#define MAX_THROTTLE (100.0f)
#define MAX_ROLL_RATE (57.3f)
#define MAX_PITCH_RATE (57.3f)

float8 simple_aircraft_model(float8 x, float4 u, float t) {
  float8 dxdt;
  float throttle = u[CONTROL_THRUST];
  float roll_rate = u[CONTROL_ROLL_RATE];
  float pitch_rate = u[CONTROL_PITCH_RATE];

  throttle = clamp(throttle, -MAX_THROTTLE, MAX_THROTTLE);
  roll_rate = clamp(roll_rate, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  pitch_rate = clamp(pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);

  float speed = x[MODEL_SPEED]; 
  float xy_speed = speed*cos(x[MODEL_PITCH]);
  dxdt[MODEL_X] = xy_speed * cos(x[MODEL_YAW]); 
  dxdt[MODEL_Y] = xy_speed * sin(x[MODEL_YAW]); 
  dxdt[MODEL_Z] = -sin(x[MODEL_PITCH])*speed; 
  dxdt[MODEL_ROLL] = roll_rate;
  dxdt[MODEL_PITCH] = pitch_rate;

  float current_length = TURNING_RADIUS + RADIUS_SLOPE_PER_SPEED * (speed - SPEED_TARGET);
  dxdt[MODEL_YAW] = speed/current_length*tan(x[MODEL_ROLL]);

  dxdt[MODEL_SPEED] = throttle / 5;

  return dxdt;
}

float8 state_to_model(float* state) {
  float8 model;
  float4 q;
  q.x = state[STATE_QUAT_X];
  q.y = state[STATE_QUAT_Y];
  q.z = state[STATE_QUAT_Z];
  q.w = state[STATE_QUAT_W];

  model[MODEL_X] = state[STATE_X];
  model[MODEL_Y] = state[STATE_Y];
  model[MODEL_Z] = state[STATE_Z];
  float speed = sqrt(pown(state[STATE_X_VEL], 2) +
                            pown(state[STATE_Y_VEL], 2) +
                            pown(state[STATE_Z_VEL], 2)); 

  model[MODEL_SPEED] = clamp(speed, MIN_SPEED, MAX_SPEED);
  model[MODEL_ROLL] = clamp(quat_roll(q), -MAX_ROLL, MAX_ROLL);
  model[MODEL_PITCH] = clamp(quat_pitch(q), -MAX_PITCH, MAX_PITCH);
  model[MODEL_YAW] = quat_yaw(q);
  return model;
}

void model_to_state(float8 model, float* state) {
  state[STATE_X] = model[MODEL_X]; 
  state[STATE_Y] = model[MODEL_Y]; 
  state[STATE_Z] = model[MODEL_Z]; 

  float speed = model[MODEL_SPEED];
  float yaw = model[MODEL_YAW];
  float pitch = model[MODEL_PITCH];
  float roll = model[MODEL_ROLL];
  float4 q = quat_from_euler(yaw, pitch, roll);
  
  state[STATE_QUAT_W] = q.w;
  state[STATE_QUAT_X] = q.x;
  state[STATE_QUAT_Y] = q.y;
  state[STATE_QUAT_Z] = q.z;

  state[STATE_X_VEL] = speed * cos(yaw) * cos(pitch);
  state[STATE_Y_VEL] = speed * sin(yaw) * cos(pitch);
  state[STATE_Z_VEL] = speed * sin(pitch);
}

__kernel void simple_aircraft(__global float* states, __global float* controls, 
                                       float t,
                                       float dt) {
  int gid, state_offset, control_offset;
  float state[STATE_NUM_PARAMS], control[CONTROL_NUM_PARAMS];
  float8 x; // Model Vector
  float4 u; // Control Vector

  gid = get_global_id(0);
  state_offset = STATE_NUM_PARAMS*gid;
  control_offset = CONTROL_NUM_PARAMS*gid;
   
  // Copy state/control information from global entity information to private vars.
  for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
    state[i] = states[state_offset + i];
  }

  for(int i = 0; i < CONTROL_NUM_PARAMS; ++i) {
    u[i] = controls[control_offset + i];
  }

  x = state_to_model(state); 
  int target_id = 1;
  // Update x with rk4
  RK4(x, u, t, dt, simple_aircraft_model);

  model_to_state(x, state);
  for(int i = 0; i < STATE_NUM_PARAMS; i++) {
    states[state_offset + i] = state[i];
  }
}
