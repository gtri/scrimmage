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

// Based on values found in SimpleAircraft.xml
__constant fp_t TURNING_RADIUS = 13.0;
__constant fp_t MIN_SPEED = 15.0;
__constant fp_t MAX_SPEED = 40.0;
__constant fp_t MAX_ROLL = RADIANS(30.0);
__constant fp_t MAX_ROLL_RATE = RADIANS(57.3);
__constant fp_t MAX_PITCH = RADIANS(30.0);
__constant fp_t MAX_PITCH_RATE = RADIANS(57.3);
__constant fp_t SPEED_TARGET = 50.0;
__constant fp_t RADIUS_SLOPE_PER_SPEED = 0.0;
__constant fp_t MAX_THROTTLE = 100.0;

fp8_t simple_aircraft_model(fp8_t x, fp8_t u, fp_t t);
fp8_t state_to_model(fp_t* state);
void model_to_state(fp8_t model, fp_t* state);

//void print_model_input(fp8_t model, fp8_t inputs, fp_t t) {
//    printf("t=%0.16f------------\nModel:\n\tX: %0.16f\n\tY: %0.16f\n\tZ: %0.16f\n\tRoll: %0.16f\n\tPitch: %0.16f\n\tYaw: %0.16f\n\tSpeed: %0.16f\n",
//        t,
//        model[SIMPLE_AIRCRAFT_MODEL_X],
//        model[SIMPLE_AIRCRAFT_MODEL_Y],
//        model[SIMPLE_AIRCRAFT_MODEL_Z],
//        model[SIMPLE_AIRCRAFT_MODEL_ROLL],
//        model[SIMPLE_AIRCRAFT_MODEL_PITCH],
//        model[SIMPLE_AIRCRAFT_MODEL_YAW],
//        model[SIMPLE_AIRCRAFT_MODEL_SPEED]);
//
//    printf("Inputs:\n\t:Thrust: %f\n\tRoll Rate: %f\n\tPitch Rate: %f\n",
//        inputs[SIMPLE_AIRCRAFT_INPUT_THRUST],
//        inputs[SIMPLE_AIRCRAFT_INPUT_ROLL_RATE],
//        inputs[SIMPLE_AIRCRAFT_INPUT_PITCH_RATE]);
//    
//}
//
//void print_state(fp_t state[STATE_NUM_PARAMS]) {
//    printf("State:\n\t"
//            "X: %f\n\t"
//            "Y: %f\n\t"
//            "Z: %f\n\t"
//            "Vx: %f\n\t"
//            "Vy: %f\n\t"
//            "Vz: %f\n\t"
//            "Qw: %f\n\t"
//            "Qx: %f\n\t"
//            "Qy: %f\n\t"
//            "Qz: %f\n\n",
//            state[STATE_X], 
//            state[STATE_Y], 
//            state[STATE_Z], 
//            state[STATE_X_VEL], 
//            state[STATE_Y_VEL], 
//            state[STATE_Z_VEL], 
//            state[STATE_QUAT_W], 
//            state[STATE_QUAT_X], 
//            state[STATE_QUAT_Y], 
//            state[STATE_QUAT_Z]);
//}


__kernel void SimpleAircraft(__global fp_t* states, __global fp_t* inputs, 
                                       fp_t t,
                                       fp_t dt,
                                       int num_entities) {
  int gid, state_offset, control_offset;
  gid = get_global_id(0);
  if(gid < num_entities) {
    fp_t state[STATE_NUM_PARAMS];
    fp_t input[SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS];
    fp8_t x; // Model Vector
    fp8_t u; // Input Vector

    state_offset = STATE_NUM_PARAMS*gid;
    control_offset = SIMPLE_AIRCRAFT_INPUT_NUM_PARAMS*gid;

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
  model[SIMPLE_AIRCRAFT_MODEL_ROLL] = clamp(-quat_roll(q), -MAX_ROLL, MAX_ROLL);
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
  quat_t q = quat_from_euler(-roll, pitch, yaw);
  
  state[STATE_QUAT_W] = q.w;
  state[STATE_QUAT_X] = q.x;
  state[STATE_QUAT_Y] = q.y;
  state[STATE_QUAT_Z] = q.z;

  state[STATE_X_VEL] = speed * cos(yaw) * cos(pitch);
  state[STATE_Y_VEL] = speed * sin(yaw) * cos(pitch);
  state[STATE_Z_VEL] = speed * sin(pitch);
}

