#include <common.h>
#include <math_utils.h>
#include <scrimmage_defs.h>

enum SingleIntegratorInputParams {
    SINGLE_INTEGRATOR_INPUT_VEL_X = 0
    SINGLE_INTEGRATOR_INPUT_VEL_Y,
    SINGLE_INTEGRATOR_INPUT_VEL_Z,
    SINGLE_INTEGRATOR_INPUT_HEADING, // Not used by default. 
    SINGLE_INTEGRATOR_INPUT_NUM_PARAMS
};

fp8_t single_integrator_model(fp8_t x, fp8_t u, fp_t t);
fp8_t state_to_model(fp_t* state);
void model_to_state(fp8_t model, fp_t* state);

__kernel void Singleintegrator(__global fp_t* states, __global fp_t* inputs, 
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
   
  // Copy state/control information from global entity information to private vars.
//  for(int i = 0; i < STATE_NUM_PARAMS; ++i) {
//    state[i] = states[state_offset + i];
//  }
//
//  for(int i = 0; i < SINGLE_INTEGRATOR_INPUT_NUM_PARAMS; ++i) {
//    u[i] = inputs[control_offset + i];
//  }

  //x = state_to_model(state); 
  double x_vel = inputs[control_offset + SINGLE_INTEGRATOR_INPUT_VEL_X];
  double y_vel = inputs[control_offset + SINGLE_INTEGRATOR_INPUT_VEL_Y];
  double z_vel = inputs[control_offset + SINGLE_INTEGRATOR_INPUT_VEL_Z];

  states[state_offset + STATE_X_VEL] = x_vel; 
  states[state_offset + STATE_Y_VEL] = y_vel; 
  states[state_offset + STATE_Z_VEL] = z_vel; 

  states[state_offset + STATE_X] += x_vel * dt;
  states[state_offset + STATE_Y] += y_vel * dt;
  states[state_offset + STATE_Z] += z_vel * dt;

  double heading = atan2(y_vel, x_vel); 
  double heading_norm = sqrt(x_vel * x_vel + y_vel * y_vel);
  double pitch = atan2(z_vel, heading_norm) 

  fp4_t quat = quat_from_euler(0, pitch, yaw);

  states[state_offset + STATE_QUAT_W] = q.w
  states[state_offset + STATE_QUAT_X] = q.x
  states[state_offset + STATE_QUAT_Y] = q.y
  states[state_offset + STATE_QUAT_Z] = q.z
}
