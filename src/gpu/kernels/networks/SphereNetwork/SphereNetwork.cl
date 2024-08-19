#pragma OPENCL EXTENSION cl_khr_fp64 : enable

__kernel void SphereNetwork(__global double* positions, __global bool reachable_map[], double range) {
    // Get worker info
    size_t n_rows = get_global_size(0);
    size_t n_cols = get_global_size(1);
    size_t row = get_global_id(0);
    size_t col = get_global_id(1);

    if (n_rows == 1 && n_cols == 1) {
        reachable_map[0] = true;
        return;
    }

    // Return if worker has nothing to do.
    if(row >= n_rows || col >= n_cols) return;

    // TODO: Create a n*(n-1)/2 array of workers (this might eventually be too large, but that can be dealt with later)
    // Need to figure out the exact mapping between the indexing of the workers in the array to each pair of agents

    bool n_rows_odd = n_rows % 2 == 1;
    bool n_cols_odd = n_cols % 2 == 1;

    size_t antidiagonal_offset = (n_cols_odd) ? 1 : 0;

    size_t num_agents = 2*n_cols + (1 - antidiagonal_offset); 
    size_t n = num_agents - 1;    

    size_t first = 0;
    size_t second = 0;
    
    if (row > n - 1 - col) {
        first = n - row; 
        second = n - 1 - col;
    } else {
        first = row;
        second = col; 
    }

    second += (1 + first);

    double3 first_pos = vload3(first, positions);
    double3 second_pos = vload3(second, positions);

    double distance_between = distance(first_pos, second_pos);

    bool within_range = distance_between <= range;

    reachable_map[first*num_agents + second] = within_range;
}
