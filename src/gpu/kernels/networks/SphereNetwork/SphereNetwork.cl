#pragma OPENCL EXTENSION cl_khr_fp64 : enable

__kernel void SphereNetwork(__global double* positions, 
                             __global bool* reachable_map, 
                             __global double* distances,
                             double range,
                             int n_rows,
                             int n_cols) {
// Get worker info
size_t row = get_global_id(0);
size_t col = get_global_id(1);

if (n_rows == 1 && n_cols == 1) {
reachable_map[0] = true;
return;
}

// Return if worker has nothing to do.

// TODO: There is something weird here
if(row >= n_rows || col >= n_cols) {
    return;
}

size_t first_ind = col;
size_t second_ind = row;

bool on_diagonal = (row == col);
bool above_diagonal = (row < col);
bool odd_num_agents = n_rows >= 2*n_cols;
bool reverse_indexing = (odd_num_agents && on_diagonal) || above_diagonal;

size_t num_agents = (odd_num_agents) ? n_rows : 2*n_cols;
size_t n = num_agents - 1;    

if (reverse_indexing) {
    if (odd_num_agents) {
        first_ind = n - 1 - col;
    } else {
        first_ind = n - col;
    }
    second_ind = n - row;
} else if (!odd_num_agents) {
    second_ind += 1;
}

double3 first_pos = vload3(first_ind, positions);
double3 second_pos = vload3(second_ind, positions);

double distance_between = distance(first_pos, second_pos);

bool within_range = distance_between <= range;

size_t index = col*n_rows + row;
reachable_map[index] = within_range;
distances[index] = group_id;
}
