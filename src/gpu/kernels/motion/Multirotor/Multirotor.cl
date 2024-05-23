#include <common.h>
#include <math_utils.h>
#include <scrimmage_defs.h>

enum MultirotorModelParams {
      MULTI_ROTOR_U = 0,
      MULTI_ROTOR_V,
      MULTI_ROTOR_W,
      MULTI_ROTOR_P,
      MULTI_ROTOR_Q,
      MULTI_ROTOR_R,
      MULTI_ROTOR_Uw,
      MULTI_ROTOR_Vw,
      MULTI_ROTOR_Ww,
      MULTI_ROTOR_Xw,
      MULTI_ROTOR_Yw,
      MULTI_ROTOR_Zw,
      MULTI_ROTOR_QW,
      MULTI_ROTOR_QX,
      MULTI_ROTOR_QY,
      MULTI_ROTOR_QZ,
      MULTI_ROTOR_MODEL_NUM_ITEMS
};

enum MultirotorInputParams {
    MULTIROTOR_INPUT_THRUST = 0,
    MULTIROTOR_INPUT_ROLL_RATE,
    MULTIROTOR_INPUT_PITCH_RATE,
    MULTIROTOR_INPUT_NUM_PARAMS
};

// Based on values found in Multirotor.h and Multirotor.xml
__constant fp_t C_D = 0.058;
__constant fp_t C_T = 5.45e-6;
__constant fp_t C_Q = 2.284e-7;
__constant fp_t W_MAX = 1200.0;
__constant fp_t W_MIN = 346.41;
__constant fp_t W_0 = 734.847;

__constant fp3_t[3] I_MAT = [{0.0122, 0., 0.},
                             {0., 0.0122, 0.},
                             {0., 0., 0.0244}];

fp8_t multirotor_model(fp8_t x, fp8_t u, fp_t t);
fp8_t state_to_model(fp_t* state);
void model_to_state(fp8_t model, fp_t* state);

__kernel void Multirotor(__global fp_t* states, __global fp_t* inputs, 
                                       fp_t t,
                                       fp_t dt) {
  int gid, state_offset, control_offset;
  fp_t state[STATE_NUM_PARAMS];
  fp_t input[MULTIROTOR_INPUT_NUM_PARAMS];
  fp8_t x; // Model Vector
  fp8_t u; // Input Vector

  gid = get_global_id(0);
  state_offset = STATE_NUM_PARAMS*gid;
  control_offset = MULTIROTOR_INPUT_NUM_PARAMS*gid;

  // Copy state/control information from global entity information to private vars.
  for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
    state[i] = states[state_offset + i];
  }

  for(int i = 0; i < MULTIROTOR_INPUT_NUM_PARAMS; ++i) {
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
  fp_t throttle = u[MULTIROTOR_INPUT_THRUST];
  fp_t roll_rate = u[MULTIROTOR_INPUT_ROLL_RATE];
  fp_t pitch_rate = u[MULTIROTOR_INPUT_PITCH_RATE];

  throttle = clamp(throttle, -MAX_THROTTLE, MAX_THROTTLE);
  roll_rate = clamp(roll_rate, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  pitch_rate = clamp(pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);

  fp_t speed = x[MULTIROTOR_MODEL_SPEED]; 
  fp_t xy_speed = speed*cos(x[MULTIROTOR_MODEL_PITCH]);
  dxdt[MULTIROTOR_MODEL_X] = xy_speed * cos(x[SIMPLE_AIRCRAFT_MODEL_YAW]); 
  dxdt[MULTIROTOR_MODEL_Y] = xy_speed * sin(x[SIMPLE_AIRCRAFT_MODEL_YAW]); 
  dxdt[MULTIROTOR_MODEL_Z] = -sin(x[SIMPLE_AIRCRAFT_MODEL_PITCH])*speed; 
  dxdt[MULTIROTOR_MODEL_ROLL] = roll_rate;
  dxdt[MULTIROTOR_MODEL_PITCH] = pitch_rate;

  fp_t current_length = TURNING_RADIUS + RADIUS_SLOPE_PER_SPEED * (speed - SPEED_TARGET);
  dxdt[MULTIROTOR_MODEL_YAW] = speed/current_length*tan(x[SIMPLE_AIRCRAFT_MODEL_ROLL]);

  dxdt[MULTIROTOR_MODEL_SPEED] = throttle / 5;

  return dxdt;
}

fp8_t state_to_model(fp_t* state) {
  fp8_t model;
  quat_t q;
  q.x = state[STATE_QUAT_X];
  q.y = state[STATE_QUAT_Y];
  q.z = state[STATE_QUAT_Z];
  q.w = state[STATE_QUAT_W];

  model[MULTIROTOR_MODEL_X] = state[STATE_X];
  model[MULTIROTOR_MODEL_Y] = state[STATE_Y];
  model[MULTIROTOR_MODEL_Z] = state[STATE_Z];
  fp_t speed = sqrt(pown(state[STATE_X_VEL], 2) +
                            pown(state[STATE_Y_VEL], 2) +
                            pown(state[STATE_Z_VEL], 2)); 

  model[MULTIROTOR_MODEL_SPEED] = clamp(speed, MIN_SPEED, MAX_SPEED);
  model[MULTIROTOR_MODEL_ROLL] = clamp(-quat_roll(q), -MAX_ROLL, MAX_ROLL);
  model[MULTIROTOR_MODEL_PITCH] = clamp(quat_pitch(q), -MAX_PITCH, MAX_PITCH);
  model[MULTIROTOR_MODEL_YAW] = quat_yaw(q);
  return model;
}

void model_to_state(fp8_t model, fp_t* state) {
  state[STATE_X] = model[MULTIROTOR_MODEL_X]; 
  state[STATE_Y] = model[MULTIROTOR_MODEL_Y]; 
  state[STATE_Z] = model[MULTIROTOR_MODEL_Z]; 

  fp_t speed = model[MULTIROTOR_MODEL_SPEED];
  fp_t yaw = model[MULTIROTOR_MODEL_YAW];
  fp_t pitch = model[MULTIROTOR_MODEL_PITCH];
  fp_t roll = model[MULTIROTOR_MODEL_ROLL];
  quat_t q = quat_from_euler(-roll, pitch, yaw);
  
  state[STATE_QUAT_W] = q.w;
  state[STATE_QUAT_X] = q.x;
  state[STATE_QUAT_Y] = q.y;
  state[STATE_QUAT_Z] = q.z;

  state[STATE_X_VEL] = speed * cos(yaw) * cos(pitch);
  state[STATE_Y_VEL] = speed * sin(yaw) * cos(pitch);
  state[STATE_Z_VEL] = speed * sin(pitch);
}

