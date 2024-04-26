float z_offset(float phase, float t) {
  return sin(phase)*sin(t);
}

__kernel void static_motion(__global float* state, __global float* control, 
                                      float t,
                                      int num_states,
                                      int num_control_inputs) {
  int gid, state_offset, control_input_offset;

  gid = get_global_id(0);
  state_offset = num_states*gid;

  float displacement_stride = 2; 
  float phase = ((float) gid) *M_PI_4_F;
  
  state[state_offset] = gid*displacement_stride;
  state[state_offset + 1] = 0;
  state[state_offset + 2] = 300 + z_offset(phase, t);
}
