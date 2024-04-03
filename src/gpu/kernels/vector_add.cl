__kernel void vector_add(__global float4 *A, __global float4 *B, __global float4 *C) {
    int i = get_global_id(0);
    C[i] = A[i] + B[i];
}
