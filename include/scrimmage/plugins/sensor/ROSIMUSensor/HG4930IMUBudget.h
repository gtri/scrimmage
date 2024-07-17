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
 * @brief Error budget generating IMU errors to emulate an HG4930 IMU
 * @section DESCRIPTION
 * Error budget generating IMU errors to emulate an HG4930 IMU
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_HG4930IMUBUDGET_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_HG4930IMUBUDGET_H_

#include <scrimmage/plugins/sensor/ROSIMUSensor/IMUErrorBudgetTemplate.h>

#include <math.h>

class HG4930IMUBudget : public IMUErrorBudgetTemplate {
 public:
    bool localInRunBiasOption = true;  // false = 1st order Gauss-Markov
                                       // process, true = Barnes-Jarvis model

    // Accelerometer Spec.'s
    double AccelFullScaleG = 20;  // +/- 30 g
    // Scale factor & misalignment
    double AccelScaleFactorPPMStdDev = 600.0;  // scale factor 1-sigma in ppm
    double AccelMisalignmentRadians = 0.0 * 1e-6;  // 100 micro-rad 1-sigma
    // Bias
    double AccelBiasRepeatabilityG = 1.7e-3;  // 1 milli-g 1-sigma
    double AccelBiasInstabilityG = 0.025e-3;  // 50 micro-g 1-sigma
    double AccelBiasTimeConstantSec =
        60.0;  // accel bias time constant in seconds
    // Velocity random walk
    double AccelVRWGPerRtHz =
        0.03 * (1.0 / 60) * 1.0 /
        9.8;  // Units of g's per root hertz -  0.065 fps/rt(hr)
    // Quantization
    double localAccelDeltaVQuantizationMPS = pow(2, -29);

    // Gyro Spec.'s
    double GyroFullScaleDegPerSec =
        400.0;  // Full scale in degrees/second +/- 1000
    // Scale factor & misalignment
    double GyroScaleFactorPPMStdDev = 600;  // 0.01% (i.e. 150 PPM) 1-sigma
    double GyroMisalignmentRadians = 0.0 * 1e-6;  // 100 arcsec 1-sigma
    // Bias
    double GyroBiasRepeatabilityDegPerHr = 7.0;  // 1.0 deg/Hr 1-sigma
    double GyroBiasInstabilityDegPerHr = 0.25;   // 1 deg/Hr 1-sigma
    double GyroBiasInstabilityTimeConstantSec =
        60.0;  // 100.0 second time constant for in-run bias instability
    // Angle random walk
    double GyroARWDegPerRtHr = 0.04;  // 0.09 deg/sqrt(hr)
    // Quantization
    double localGyroDeltaThetaQuantizationRadians = pow(2, -33);

    HG4930IMUBudget();
    void CalculateParameters();
};

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_HG4930IMUBUDGET_H_
