#ifndef GPU_KERNELS_MOTION_MODELS_SIMPLE_AIRCRAFT_SIMPLE_AIRCRAFT_H
#define GPU_KERNELS_MOTION_MODELS_SIMPLE_AIRCRAFT_SIMPLE_AIRCRAFT_H

enum StateParams {
  STATE_X = 0,
  STATE_Y,
  STATE_Z,
  STATE_X_VEL,
  STATE_Y_VEL,
  STATE_Z_VEL,
  STATE_X_ANG_VEL,
  STATE_Y_ANG_VEL,
  STATE_Z_ANG_VEL,
  STATE_QUAT_W,
  STATE_QUAT_X,
  STATE_QUAT_Y,
  STATE_QUAT_Z,
  STATE_NUM_PARAMS
};

enum ModelParams {
    MODEL_X = 0,
    MODEL_Y,
    MODEL_Z,
    MODEL_ROLL,
    MODEL_PITCH,
    MODEL_YAW,
    MODEL_SPEED,
    MODEL_NUM_PARAMS
};

enum ControlParams {
    INPUT_THRUST = 0,
    INPUT_ROLL_RATE,
    INPUT_PITCH_RATE,
    INPUT_NUM_PARAMS
};

#define MAX_SPEED (40.0f)
#define MIN_SPEED (15.0f)
#define MAX_ROLL (30.0f)
#define MAX_PITCH (30.0f)
#define TURNING_RADIUS (50.0f)
#define SPEED_TARGET (50.0f)
#define RADIUS_SLOPE_PER_SPEED (0.0f)
#define MAX_THROTTLE (100.0f)
#define MAX_ROLL_RATE (57.3f)
#define MAX_PITCH_RATE (57.3f)

float8 simple_aircraft_model(float8 x, float4 u, float t);

float8 state_to_model(float* state);

void model_to_state(float8 model, float* state);

#endif //GPU_KERNELS_MOTION_MODELS_SIMPLE_AIRCRAFT_SIMPLE_AIRCRAFT_H
