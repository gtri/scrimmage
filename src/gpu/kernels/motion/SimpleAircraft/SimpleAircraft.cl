#include <common.h>
#include <math_utils.h>
#include <scrimmage_defs.h>

enum SimpleAircraftModelParams {
    SIMPLE_AIRCRAFT_MODEL_X = 0,
    SIMPLE_AIRCRAFT_MODEL_Y,
    SIMPLE_AIRCRAFT_MODEL_Z,
    SIMPLE_AIRCRAFT_MODEL_ROLL,
    SIMPLE_AIRCRAFT_MODEL_PITCH,
    SIMPLE_AIRCRAFT_MODEL_YAW,
    SIMPLE_AIRCRAFT_MODEL_SPEED,
    SIMPLE_AIRCRAFT_MODEL_NUM_PARAMS
};

enum SimpleAircraftInputParams {
    SIMPLE_AIRCRAFT_INPUT_THRUST = 0,
    SIMPLE_AIRCRAFT_INPUT_ROLL_RATE,
    SIMPLE_AIRCRAFT_INPUT_PITCH_RATE,
    SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS
};

#define MAX_SPEED ((fp_t) 40.0f)
#define MIN_SPEED ((fp_t) 15.0f)
#define MAX_ROLL ((fp_t) 30.0f)
#define MAX_PITCH ((fp_t) 30.0f)
#define TURNING_RADIUS ((fp_t) 50.0f)
#define SPEED_TARGET ((fp_t) 50.0f)
#define RADIUS_SLOPE_PER_SPEED ((fp_t) 0.0f)
#define MAX_THROTTLE ((fp_t) 100.0f)
#define MAX_ROLL_RATE ((fp_t) 57.3f)
#define MAX_PITCH_RATE ((fp_t) 57.3f)

fp8_t simple_aircraft_model(fp8_t x, fp8_t u, fp_t t);
fp8_t state_to_model(fp_t* state);
void model_to_state(fp8_t model, fp_t* state);

__kernel void SimpleAircraft(__global fp_t* states, __global fp_t* inputs, 
                                       fp_t t,
                                       fp_t dt) {
  int gid, state_offset, control_offset;
  fp_t state[STATE_NUM_PARAMS];
  fp_t input[SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS];
  fp8_t x; // Model Vector
  fp8_t u; // Input Vector

  gid = get_global_id(0);
  state_offset = STATE_NUM_PARAMS*gid;
  control_offset = SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS*gid;

  printf("Hello"); // Try this?

  // Copy state/control information from global entity information to private vars.
  for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
    state[i] = states[state_offset + i];
  }

  for(int i = 0; i < SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS; ++i) {
    u[i] = inputs[control_offset + i];
  }
  x = state_to_model(state); 

  // Update x with rk4
  RK4(x, u, t, dt, simple_aircraft_model);

  model_to_state(x, state);
  for(int i = 0; i < STATE_NUM_PARAMS; i++) {
    states[state_offset + i] = state[i];
  }
}

fp8_t simple_aircraft_model(fp8_t x, fp8_t u, fp_t t) {
  fp8_t dxdt;
  fp_t throttle = u[SIMPLE_AIRCRAFT_INPUT_THRUST];
  fp_t roll_rate = u[SIMPLE_AIRCRAFT_INPUT_ROLL_RATE];
  fp_t pitch_rate = u[SIMPLE_AIRCRAFT_INPUT_PITCH_RATE];

  throttle = clamp(throttle, -MAX_THROTTLE, MAX_THROTTLE);
  roll_rate = clamp(roll_rate, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  pitch_rate = clamp(pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);

  fp_t speed = x[SIMPLE_AIRCRAFT_MODEL_SPEED]; 
  fp_t xy_speed = speed*cos(x[SIMPLE_AIRCRAFT_MODEL_PITCH]);
  dxdt[SIMPLE_AIRCRAFT_MODEL_X] = xy_speed * cos(x[SIMPLE_AIRCRAFT_MODEL_YAW]); 
  dxdt[SIMPLE_AIRCRAFT_MODEL_Y] = xy_speed * sin(x[SIMPLE_AIRCRAFT_MODEL_YAW]); 
  dxdt[SIMPLE_AIRCRAFT_MODEL_Z] = -sin(x[SIMPLE_AIRCRAFT_MODEL_PITCH])*speed; 
  dxdt[SIMPLE_AIRCRAFT_MODEL_ROLL] = roll_rate;
  dxdt[SIMPLE_AIRCRAFT_MODEL_PITCH] = pitch_rate;

  fp_t current_length = TURNING_RADIUS + RADIUS_SLOPE_PER_SPEED * (speed - SPEED_TARGET);
  dxdt[SIMPLE_AIRCRAFT_MODEL_YAW] = speed/current_length*tan(x[SIMPLE_AIRCRAFT_MODEL_ROLL]);

  dxdt[SIMPLE_AIRCRAFT_MODEL_SPEED] = throttle / 5;

  return dxdt;
}

fp8_t state_to_model(fp_t* state) {
  fp8_t model;
  quat_t q;
  q.x = state[STATE_QUAT_X];
  q.y = state[STATE_QUAT_Y];
  q.z = state[STATE_QUAT_Z];
  q.w = state[STATE_QUAT_W];

  model[SIMPLE_AIRCRAFT_MODEL_X] = state[STATE_X];
  model[SIMPLE_AIRCRAFT_MODEL_Y] = state[STATE_Y];
  model[SIMPLE_AIRCRAFT_MODEL_Z] = state[STATE_Z];
  fp_t speed = sqrt(pown(state[STATE_X_VEL], 2) +
                            pown(state[STATE_Y_VEL], 2) +
                            pown(state[STATE_Z_VEL], 2)); 

  model[SIMPLE_AIRCRAFT_MODEL_SPEED] = clamp(speed, MIN_SPEED, MAX_SPEED);
  model[SIMPLE_AIRCRAFT_MODEL_ROLL] = clamp(quat_roll(q), -MAX_ROLL, MAX_ROLL);
  model[SIMPLE_AIRCRAFT_MODEL_PITCH] = clamp(quat_pitch(q), -MAX_PITCH, MAX_PITCH);
  model[SIMPLE_AIRCRAFT_MODEL_YAW] = quat_yaw(q);
  return model;
}

void model_to_state(fp8_t model, fp_t* state) {
  state[STATE_X] = model[SIMPLE_AIRCRAFT_MODEL_X]; 
  state[STATE_Y] = model[SIMPLE_AIRCRAFT_MODEL_Y]; 
  state[STATE_Z] = model[SIMPLE_AIRCRAFT_MODEL_Z]; 

  fp_t speed = model[SIMPLE_AIRCRAFT_MODEL_SPEED];
  fp_t yaw = model[SIMPLE_AIRCRAFT_MODEL_YAW];
  fp_t pitch = model[SIMPLE_AIRCRAFT_MODEL_PITCH];
  fp_t roll = model[SIMPLE_AIRCRAFT_MODEL_ROLL];
  quat_t q = quat_from_euler(yaw, pitch, roll);
  
  state[STATE_QUAT_W] = q.w;
  state[STATE_QUAT_X] = q.x;
  state[STATE_QUAT_Y] = q.y;
  state[STATE_QUAT_Z] = q.z;

  state[STATE_X_VEL] = speed * cos(yaw) * cos(pitch);
  state[STATE_Y_VEL] = speed * sin(yaw) * cos(pitch);
  state[STATE_Z_VEL] = speed * sin(pitch);
}

