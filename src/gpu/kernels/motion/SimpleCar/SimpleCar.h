#ifndef GPU_KERNELS_MOTION_MODELS_SIMPLE_CAR_SIMPLE_CAR_H
#define GPU_KERNELS_MOTION_MODELS_SIMPLE_CAR_SIMPLE_CAR_H

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
    MODEL_Z_DOT,
    MODEL_THETA,
    MODEL_NUM_PARAMS
};

enum InputParams {
    INPUT_FORWARD_VELOCITY = 0,
    INPUT_TURN_RATE,
    INPUT_NUM_PARAMS
};

#define MAX_SPEED (30.0f)
#define MAX_OMEGA (M_PI_4_F)
#define LENGTH (5.0f)
#define MASS (1.0f)

float8 simple_car_model(float8 x, float4 input, float t);

float8 state_to_model(float* state);

void model_to_state(float8 model, float8 dxdt, float* state);

#endif //GPU_KERNELS_MOTION_MODELS_SIMPLE_CAR_SIMPLE_CAR_H
