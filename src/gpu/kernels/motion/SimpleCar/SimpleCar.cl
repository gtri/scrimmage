#include <common.h>
#include <math_utils.h>
#include <scrimmage_defs.h>

enum ModelParams {
    SIMPLED_CAR_MODEL_X = 0,
    SIMPLED_CAR_MODEL_Y,
    SIMPLED_CAR_MODEL_Z,
    SIMPLED_CAR_MODEL_Z_DOT,
    SIMPLED_CAR_MODEL_THETA,
    SIMPLED_CAR_MODEL_NUM_PARAMS,
};

enum InputParams {
    SIMPLED_CAR_INPUT_FORWARD_VELOCITY = 0,
    SIMPLED_CAR_INPUT_TURN_RATE,
    SIMPLED_CAR_INPUT_NUM_PARAMS
};

#define MAX_SPEED (30.0f)
#define MAX_OMEGA (M_PI_4_F)
#define LENGTH (5.0f)
#define MASS (1.0f)

float8 simple_car_model(float8 x, float4 input, float t);
float8 state_to_model(float* state);
void model_to_state(float8 model, float8 dxdt, float* state);

__kernel void SimpleCar(__global float* states, __global float* inputs, 
                                       float t,
                                       float dt) {
  int gid, state_offset, input_offset;
  float state[STATE_NUM_PARAMS], input[SIMPLE_CAR_INPUT_NUM_PARAMS];
  float8 x; // Model Vector
  float4 u; // Input Vector

  gid = get_global_id(0);
  state_offset = STATE_NUM_PARAMS*gid;
  input_offset = INPUT_NUM_PARAMS*gid;
   
  // Copy state/control information from global entity information to private vars.
  for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
    state[i] = states[state_offset + i];
  }

  for(int i = 0; i < INPUT_NUM_PARAMS; ++i) {
    u[i] = inputs[input_offset + i];
  }

  x = state_to_model(state); 
  int target_id = 1;
  // Update x with rk4
  RK4(x, u, t, dt, simple_car_model);

  model_to_state(x, simple_car_model(x, u, t+dt), state);
  for(int i = 0; i < STATE_NUM_PARAMS; i++) {
    states[state_offset + i] = state[i];
  }
}

float8 simple_car_model(float8 x, float4 input, float t) {
  float8 dxdt;
  float speed = clamp(input[SIMPLE_CAR_INPUT_FORWARD_VELOCITY], -MAX_SPEED, MAX_SPEED);
  float omega = clamp(input[SIMPLE_CAR_INPUT_TURN_RATE], -MAX_OMEGA, MAX_OMEGA);
  float theta = x[SIMPLE_CAR_MODEL_THETA];
  
  dxdt[SIMPLE_CAR_MODEL_X] = speed * cos(theta);
  dxdt[SIMPLE_CAR_MODEL_Y] = speed * sin(theta);
  dxdt[SIMPLE_CAR_MODEL_THETA] = speed / LENGTH * tan(theta);
  dxdt[SIMPLE_CAR_MODEL_Z] = 0;
  dxdt[SIMPLE_CAR_MODEL_Z_DOT] = 0;

  return dxdt;
}

float8 state_to_model(float* state) {
  float8 model;
  float4 q;
  q.x = state[STATE_QUAT_X];
  q.y = state[STATE_QUAT_Y];
  q.z = state[STATE_QUAT_Z];
  q.w = state[STATE_QUAT_W];

  model[SIMPLE_CAR_MODEL_X] = state[STATE_X];
  model[SIMPLE_CAR_MODEL_Y] = state[STATE_Y];
  model[SIMPLE_CAR_MODEL_Z] = state[STATE_Z];
  model[SIMPLE_CAR_MODEL_Z_DOT] = state[STATE_Z_VEL];
  model[SIMPLE_CAR_MODEL_THETA] = quat_yaw(q);
  return model;
}

void model_to_state(float8 model, float8 dxdt, float* state) {
  state[STATE_X] = model[SIMPLE_CAR_MODEL_X]; 
  state[STATE_Y] = model[SIMPLE_CAR_MODEL_Y]; 
  state[STATE_Z] = model[SIMPLE_CAR_MODEL_Z]; 
  
  state[STATE_X_VEL] = dxdt[SIMPLE_CAR_MODEL_X]; 
  state[STATE_Y_VEL] = dxdt[SIMPLE_CAR_MODEL_Y];
  state[STATE_Z_VEL] = model[SIMPLE_CAR_MODEL_Z_DOT];

  state[STATE_Z_ANG_VEL] = dxdt[SIMPLE_CAR_MODEL_THETA];

  float4 q = quat_from_euler(0, 0, model[SIMPLE_CAR_MODEL_THETA]); 

  state[STATE_QUAT_W] = q.w;
  state[STATE_QUAT_X] = q.x;
  state[STATE_QUAT_Y] = q.y;
  state[STATE_QUAT_Z] = q.z;
}

