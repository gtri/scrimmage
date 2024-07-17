/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author David Burke <david.burke@gtri.gatech.edu>
 * @date 28 May 2020
 * @version 0.1.0
 * @brief Base class for IMU Error budget
 * @section DESCRIPTION
 * Base class for IMU Error budget
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORBUDGETTEMPLATE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORBUDGETTEMPLATE_H_

class IMUErrorBudgetTemplate {
 public:
    double SampleFrequency;
    bool InRunBiasOption;  // 0 = 1st order Gauss-Markov process, 1 =
                           // Barnes-Jarvis model
    // Accelerometer Spec.'s
    // Scale factor
    double XAccelAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    double YAccelAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    double ZAccelAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    // Scale factor Nonlinearity
    double XAccelAxisScaleFactorNonlinearityMPS2PerMPS22StdDev =
        0.0;  // 1-sigma in units of PPM
    double YAccelAxisScaleFactorNonlinearityMPS2PerMPS22StdDev =
        0.0;  // 1-sigma in units of PPM
    double ZAccelAxisScaleFactorNonlinearityMPS2PerMPS22StdDev =
        0.0;  // 1-sigma in units of PPM
    // Misalignment
    double AccelMisalignmentPPMStdDev = 0.0;  // 1-sigma in units of PPM
    // Turn-on Bias
    double XAccelAxisTurnOnBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared
    double YAccelAxisTurnOnBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared
    double ZAccelAxisTurnOnBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared
    // In-run Bias
    double XAccelAxisInRunBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared
    double YAccelAxisInRunBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared
    double ZAccelAxisInRunBiasMPS2StdDev =
        0.0;  // 1-sigma in units of meters per second (MPS) squared

    double XAccelAxisInRunBiasTimeConstantSec =
        100.0;  // in-run bias time constant in units of seconds
    double YAccelAxisInRunBiasTimeConstantSec =
        100.0;  // in-run bias time constant in units of seconds
    double ZAccelAxisInRunBiasTimeConstantSec =
        100.0;  // in-run bias time constant in units of seconds

    double XAccelAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    double YAccelAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    double ZAccelAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    // Velocity random walk
    double XAccelVRWMPS2PerRtHzStdDev =
        0.0;  // velocity random walk (VRW) 1-sigma in units of meters per
              // second (MPS) squared
    double YAccelVRWMPS2PerRtHzStdDev =
        0.0;  // velocity random walk (VRW) 1-sigma in units of meters per
              // second (MPS) squared
    double ZAccelVRWMPS2PerRtHzStdDev =
        0.0;  // velocity random walk (VRW) 1-sigma in units of meters per
              // second (MPS) squared
    // Quantization
    double AccelDeltaVQuantizationMPS;

    // Gyro Spec.'s
    // Scale factor
    double XGyroAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    double YGyroAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    double ZGyroAxisScaleFactorPPMStdDev = 0.0;  // 1-sigma in units of PPM
    // Misalignment
    double GyroMisalignmentPPMStdDev = 0.0;  // 1-sigma in units of PPM
    // Turn-on bias
    double XGyroAxisTurnOnBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second
    double YGyroAxisTurnOnBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second
    double ZGyroAxisTurnOnBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second
    // In-run bias
    double XGyroAxisInRunBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second
    double YGyroAxisInRunBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second
    double ZGyroAxisInRunBiasRadPerSecStdDev =
        0.0;  // 1-sigma in units of radians/second

    double XGyroAxisInRunBiasTimeConstantSec =
        300.0;  // in-run bias time constant in units of seconds
    double YGyroAxisInRunBiasTimeConstantSec =
        300.0;  // in-run bias time constant in units of seconds
    double ZGyroAxisInRunBiasTimeConstantSec =
        300.0;  // in-run bias time constant in units of seconds

    double XGyroAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    double YGyroAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    double ZGyroAxisInRunBiasContinuousProcessNoiseVar =
        0.0;  // variance of continuous-time process noise driving in-run bias
    // Angle random walk
    double XGyroARWRadPerSecPerRtHzStdDev =
        0.0;  // angular random walk (ARW) 1-sigma in units of radians/sec
    double YGyroARWRadPerSecPerRtHzStdDev =
        0.0;  // angular random walk (ARW) 1-sigma in units of radians/sec
    double ZGyroARWRadPerSecPerRtHzStdDev =
        0.0;  // angular random walk (ARW) 1-sigma in units of radians/sec
    // Quantization
    double GyroDeltaThetaQuantizationRadians;
};

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORBUDGETTEMPLATE_H_
