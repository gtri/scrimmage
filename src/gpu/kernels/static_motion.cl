__kernel void static_motion(__global float* state, __global float* control, 
                                      int num_states,
                                      int num_control_inputs) {
  int gid, state_offset, control_input_offset;

  gid = get_global_id(0);
  state_offset = num_states*gid;
  
  for(int i = 0; i < num_states; i++) {
    state[state_offset + i] = gid;
  }
}
