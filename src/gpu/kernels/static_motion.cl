// Simply copy the state from input to output
__kernel void static_motion(__global double* state, __global double* control, 
                                      int num_inputs,
                                      int num_outputs) {
  int gid = get_global_id(0);

  int state_offset = gid*num_inputs;
  int control_offset = gid*num_outputs;
}
