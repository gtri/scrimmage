#ifndef KERNEL_COMMON_H
#define KERNEL_COMMON_H
  #pragma OPENCL EXTENSION cl_khr_fp64 : enable
  
  #ifdef SINGLE_PRECISION
    typedef float   fp_t;
    typedef float2  fp2_t;
    typedef float3  fp3_t;
    typedef float4  fp4_t;
    typedef float8  fp8_t;
  #else
    typedef double  fp_t;
    typedef double2 fp2_t;
    typedef double3 fp3_t;
    typedef double4 fp4_t;
    typedef double8 fp8_t;
  #endif
#endif
