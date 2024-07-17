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

#include <scrimmage/plugins/sensor/ROSIMUSensor/HG4930IMUBudget.h>

HG4930IMUBudget::HG4930IMUBudget() {
    InRunBiasOption = localInRunBiasOption;
    CalculateParameters();
}

void HG4930IMUBudget::CalculateParameters() {
    //// Accel. Parameters:
    // Scale factor
    XAccelAxisScaleFactorPPMStdDev =
        AccelScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    YAccelAxisScaleFactorPPMStdDev =
        AccelScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    ZAccelAxisScaleFactorPPMStdDev =
        AccelScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    // Misalignment
    AccelMisalignmentPPMStdDev =
        (1e6) * sin(AccelMisalignmentRadians);  // 1-sigma in units of PPM
    // Turn-on Bias
    XAccelAxisTurnOnBiasMPS2StdDev =
        AccelBiasRepeatabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared
    YAccelAxisTurnOnBiasMPS2StdDev =
        AccelBiasRepeatabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared
    ZAccelAxisTurnOnBiasMPS2StdDev =
        AccelBiasRepeatabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared
    // In-run Bias
    XAccelAxisInRunBiasMPS2StdDev =
        AccelBiasInstabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared
    YAccelAxisInRunBiasMPS2StdDev =
        AccelBiasInstabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared
    ZAccelAxisInRunBiasMPS2StdDev =
        AccelBiasInstabilityG *
        (9.8);  // 1-sigma in units of meters per second (MPS) squared

    XAccelAxisInRunBiasTimeConstantSec =
        AccelBiasTimeConstantSec;  // in-run bias time constant in units of
                                   // seconds
    YAccelAxisInRunBiasTimeConstantSec =
        AccelBiasTimeConstantSec;  // in-run bias time constant in units of
                                   // seconds
    ZAccelAxisInRunBiasTimeConstantSec =
        AccelBiasTimeConstantSec;  // in-run bias time constant in units of
                                   // seconds

    ////
    //
    // $$E\{ w^{2}_{a} \} = \frac{2 \sigma^{2}_{b_{a,1}}}{\tau_{a}}$$
    //

    XAccelAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(XAccelAxisInRunBiasMPS2StdDev, 2)) /
        XAccelAxisInRunBiasTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias
    YAccelAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(YAccelAxisInRunBiasMPS2StdDev, 2)) /
        YAccelAxisInRunBiasTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias
    ZAccelAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(ZAccelAxisInRunBiasMPS2StdDev, 2)) /
        ZAccelAxisInRunBiasTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias
    // Velocity random walk
    XAccelVRWMPS2PerRtHzStdDev =
        (9.8 * AccelVRWGPerRtHz);  // velocity random walk (VRW) 1-sigma in
                                   // units of meters per second (MPS) squared
    YAccelVRWMPS2PerRtHzStdDev =
        (9.8 * AccelVRWGPerRtHz);  // velocity random walk (VRW) 1-sigma in
                                   // units of meters per second (MPS) squared
    ZAccelVRWMPS2PerRtHzStdDev =
        (9.8 * AccelVRWGPerRtHz);  // velocity random walk (VRW) 1-sigma in
                                   // units of meters per second (MPS) squared
    // Quantization
    AccelDeltaVQuantizationMPS = localAccelDeltaVQuantizationMPS;

    //// Gyro Parameters:
    // Scale factor
    XGyroAxisScaleFactorPPMStdDev =
        GyroScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    YGyroAxisScaleFactorPPMStdDev =
        GyroScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    ZGyroAxisScaleFactorPPMStdDev =
        GyroScaleFactorPPMStdDev;  // 1-sigma in units of PPM
    // Misalignment
    GyroMisalignmentPPMStdDev =
        (1e6) * sin(GyroMisalignmentRadians);  // 1-sigma in units of PPM
    // Turn-on Bias
    double GyroBiasStdDevRadPerSec =
        GyroBiasRepeatabilityDegPerHr * (M_PI / 180.0) *
        (1.0 / 3600.0);  // convert to 1-sigma in radians/sec
    // Now load into appropriate variables
    XGyroAxisTurnOnBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second
    YGyroAxisTurnOnBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second
    ZGyroAxisTurnOnBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second
    // In-run Bias
    GyroBiasStdDevRadPerSec =
        GyroBiasInstabilityDegPerHr * (M_PI / 180.0) *
        (1.0 / 3600.0);  // convert to 1-sigma in radians/sec
    // Now load into appropriate variables
    XGyroAxisInRunBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second
    YGyroAxisInRunBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second
    ZGyroAxisInRunBiasRadPerSecStdDev =
        GyroBiasStdDevRadPerSec;  // 1-sigma in units of radians/second

    XGyroAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(XGyroAxisInRunBiasRadPerSecStdDev, 2)) /
        GyroBiasInstabilityTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias
    YGyroAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(YGyroAxisInRunBiasRadPerSecStdDev, 2)) /
        GyroBiasInstabilityTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias
    ZGyroAxisInRunBiasContinuousProcessNoiseVar =
        (2 * pow(ZGyroAxisInRunBiasRadPerSecStdDev, 2)) /
        GyroBiasInstabilityTimeConstantSec;  // variance of continuous-time
                                             // process noise driving in-run
                                             // bias

    // Now load time constants
    XGyroAxisInRunBiasTimeConstantSec =
        GyroBiasInstabilityTimeConstantSec;  // in-run bias time constant in
                                             // units of seconds
    YGyroAxisInRunBiasTimeConstantSec =
        GyroBiasInstabilityTimeConstantSec;  // in-run bias time constant in
                                             // units of seconds
    ZGyroAxisInRunBiasTimeConstantSec =
        GyroBiasInstabilityTimeConstantSec;  // in-run bias time constant in
                                             // units of seconds

    // Now do conversions for Angle Random Walk
    double GyroARWStdDevRadPerSecPerRtHz =
        GyroARWDegPerRtHr * (M_PI / 180.0) *
        (1.0 / 60.0);  // 1-sigma in radians/second
    // Now load into appropriate variables
    XGyroARWRadPerSecPerRtHzStdDev =
        GyroARWStdDevRadPerSecPerRtHz;  // 1-sigma in units of radians/sec
    YGyroARWRadPerSecPerRtHzStdDev =
        GyroARWStdDevRadPerSecPerRtHz;  // 1-sigma in units of radians/sec
    ZGyroARWRadPerSecPerRtHzStdDev =
        GyroARWStdDevRadPerSecPerRtHz;  // 1-sigma in units of radians/sec
    // Quantization
    GyroDeltaThetaQuantizationRadians = localGyroDeltaThetaQuantizationRadians;
}
