#include <common.h>
#include <math_utils.h>
#include <scrimmage_defs.h>

enum (>>>KERNEL_NAME<<<)_ModelParams {
  /*
  * ---------------------------------------------------------
  * !! TODO: Add additional enums for each Model Parameter !!
  * ---------------------------------------------------------
  */

  (>>>KERNEL_NAME_UPPER_SNAKE<<<)_MODEL_NUM_PARAMS 
};

enum (>>>KERNEL_NAME<<<)_InputParams {
  /*
  * ---------------------------------------------------------
  * !! TODO: Add additional enums for each Input Parameter !!
  * ---------------------------------------------------------
  */
    (>>>KERNEL_NAME_UPPER_SNAKE<<<)_INPUT_NUM_PARAMS
};

/* 
* -------------------------------------------------------
* !! TODO: ADD ANY CONSTANT PARAMETERS. EXAMPLES BELOW !!
* -------------------------------------------------------
*__constant fp_t TURNING_RADIUS = 13.0;
*__constant fp_t MIN_SPEED = 15.0;
*__constant fp_t MAX_SPEED = 40.0;
*__constant fp_t MAX_ROLL = RADIANS(30.0);
*__constant fp_t MAX_ROLL_RATE = RADIANS(57.3);
*__constant fp_t MAX_PITCH = RADIANS(30.0);
*__constant fp_t MAX_PITCH_RATE = RADIANS(57.3);
*__constant fp_t SPEED_TARGET = 50.0;
*__constant fp_t RADIUS_SLOPE_PER_SPEED = 0.0;
*__constant fp_t MAX_THROTTLE = 100.0;
*/

/*
* ------------------------------------------------------------------------------------
* !! TODO: Check if you can use different vector sizes for models/input information !!
* ------------------------------------------------------------------------------------
*/

fp8_t (>>>KERNEL_NAME_LOWER_SNAKE<<<)_model(fp8_t x, fp8_t u, fp_t t);
fp8_t state_to_model(fp_t* state);
void model_to_state(fp8_t model, fp_t* state);


__kernel void (>>>KERNEL_NAME<<<)(__global fp_t* states, __global fp_t* inputs, 
                                       fp_t t,
                                       fp_t dt,
                                       int num_entities) {
  int gid, state_offset, control_offset;
  gid = get_global_id(0);

  // Avoid computations for work-items not associated with any entity.
  if(gid < num_entities) {
    fp_t state[STATE_NUM_PARAMS];
    fp_t input[(>>>KERNEL_NAME_UPPER_SNAKE<<<)_INPUT_NUM_PARAMS];
    fp8_t x; // Model Vector
    fp8_t u; // Input Vector

    state_offset = STATE_NUM_PARAMS*gid;
    control_offset = (>>>KERNEL_NAME_UPPER_SNAKE<<<)_INPUT_NUM_PARAMS*gid;

    // Copy state/control information from global entity information to private vars.
    for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
      state[i] = states[state_offset + i];
    }

    for(int i = 0; i < (>>>KERNEL_NAME_UPPER_SNAKE<<<)_INPUT_NUM_PARAMS; ++i) {
      u[i] = inputs[control_offset + i];
    }
    x = state_to_model(state); 

    // Update x with rk4
    RK4(x, u, t, dt, (>>>KERNEL_NAME_LOWER_SNAKE<<<)_model);

    model_to_state(x, state);
    for(int i = 0; i < STATE_NUM_PARAMS; i++) {
      states[state_offset + i] = state[i];
    }
  }
}

fp8_t (>>>KERNEL_NAME_LOWER_SNAKE<<<)_model(fp8_t x, fp8_t u, fp_t t) {
  fp8_t dxdt;
  
  /*
  * -----------------------------------------------------------
  * !! TODO: Implement Motion Model (i.e. dxdt = f(x, y, t)) !!
  * -----------------------------------------------------------
  */

  return dxdt;
}

fp8_t state_to_model(fp_t* state) {
  fp8_t model;
  /*
  * ----------------------------------------------------------------
  * !! TODO: Convert State Information to Motion Model Parameters !!
  * ----------------------------------------------------------------
  */

  return model;
}

void model_to_state(fp8_t model, fp_t* state) {
  /*
  * ----------------------------------------------------------
  * !! TODO: Convert Motion Model back to State Information !!
  * ----------------------------------------------------------
  */
}

