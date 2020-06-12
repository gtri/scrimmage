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
 * @brief Class for simulating randomized IMU error 
 * @section DESCRIPTION
 * Class for adding realistic error/noise to 'perfect' IMU data generated from position, velocity, orientation data
 *
 */

#include <scrimmage/plugins/sensor/ROSIMUSensor/IMUErrorSimulator.h>

IMUErrorSimulator::IMUErrorSimulator(IMUErrorBudgetTemplate errorBudget) {
    InRunBiasOption = false;
    double randnSeedStart = InitRandomNumberSeed;
    PerformInitialRandomDraws(errorBudget);
    CalculateParametersForFixedDeltaT(errorBudget);
}

//generate a 3d vector with the passed in random distribution and generator
Eigen::Vector3d IMUErrorSimulator::RandomVector(std::normal_distribution<double> &distribution, std::mt19937 &generator)
{
    Eigen::Vector3d newVec;
    newVec << distribution(generator), distribution(generator), distribution(generator);
    //cout << "Random Vector: " << newVec.x() << "," << newVec.y() << "," << newVec.z() << endl;
    return newVec;
}

void IMUErrorSimulator::PerformInitialRandomDraws(IMUErrorBudgetTemplate errorBudget)
{
    // Accel Error Terms
    // Get random draws for accel scale factor errors:
    AccelScaleFactorErr.x() = (1e-6) * errorBudget.XAccelAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    AccelScaleFactorErr.y() = (1e-6) * errorBudget.YAccelAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    AccelScaleFactorErr.z() = (1e-6) * errorBudget.ZAccelAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    // Put scale factor errors in the matrix Ma:
    Ma = AccelScaleFactorErr.asDiagonal();

    // Do initial random draws for accel turn-on bias:
    AccelBiasTurnOn.x() = errorBudget.XAccelAxisTurnOnBiasMPS2StdDev * InitDist(InitRNG);
    AccelBiasTurnOn.y() = errorBudget.YAccelAxisTurnOnBiasMPS2StdDev * InitDist(InitRNG);
    AccelBiasTurnOn.z() = errorBudget.ZAccelAxisTurnOnBiasMPS2StdDev * InitDist(InitRNG);

    // Gyro Error Terms
    // Get random draws for gyro scale factor errors:
    GyroScaleFactorErr.x() = (1e-6) * errorBudget.XGyroAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    GyroScaleFactorErr.y() = (1e-6) * errorBudget.YGyroAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    GyroScaleFactorErr.z() = (1e-6) * errorBudget.ZGyroAxisScaleFactorPPMStdDev * InitDist(InitRNG);
    // Place gyro scale factor errors in matrix Mg:
    Mg = GyroScaleFactorErr.asDiagonal();

    // Get random draws for gyro turn-on biases
    GyroBiasTurnOn.x() = errorBudget.XGyroAxisTurnOnBiasRadPerSecStdDev * InitDist(InitRNG);
    GyroBiasTurnOn.y() = errorBudget.YGyroAxisTurnOnBiasRadPerSecStdDev * InitDist(InitRNG);
    GyroBiasTurnOn.z() = errorBudget.ZGyroAxisTurnOnBiasRadPerSecStdDev * InitDist(InitRNG);
}

void IMUErrorSimulator::CalculateParametersForFixedDeltaT(IMUErrorBudgetTemplate errorBudget)
{
    // Calculate sample interval
    NominalDeltaT = 1.0 / errorBudget.SampleFrequency;

    // Accel Error Terms:
    // The parameters below are for the accel in-run bias error
    if (InRunBiasOption == false)
    {
        AccelBiasPhi.x() = exp(-NominalDeltaT / errorBudget.XAccelAxisInRunBiasTimeConstantSec);
        AccelBiasPhi.y() = exp(-NominalDeltaT / errorBudget.YAccelAxisInRunBiasTimeConstantSec);
        AccelBiasPhi.z() = exp(-NominalDeltaT / errorBudget.ZAccelAxisInRunBiasTimeConstantSec);

        AccelBiasInRunStdDev.x() = errorBudget.XAccelAxisInRunBiasMPS2StdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.XAccelAxisInRunBiasTimeConstantSec));
        AccelBiasInRunStdDev.y() = errorBudget.YAccelAxisInRunBiasMPS2StdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.YAccelAxisInRunBiasTimeConstantSec));
        AccelBiasInRunStdDev.z() = errorBudget.ZAccelAxisInRunBiasMPS2StdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.ZAccelAxisInRunBiasTimeConstantSec));
        // do initial draw for in-run bias
        //                AccelBiasInRun = diag([errorBudget.XAccelAxisInRunBiasMPS2StdDev;errorBudget.YAccelAxisInRunBiasMPS2StdDev;errorBudget.ZAccelAxisInRunBiasMPS2StdDev])*randn(AccelBiasInRunRandomNumberStreamPtr,3,1);;

        AccelBiasInRun = Eigen::Vector3d(errorBudget.XAccelAxisInRunBiasMPS2StdDev, errorBudget.YAccelAxisInRunBiasMPS2StdDev, errorBudget.ZAccelAxisInRunBiasMPS2StdDev).asDiagonal() * RandomVector(AccelBiasDist, AccelBiasRNG);
    }
    // Following is the standard deviation of the velocity random walk noise
    AccelVRWMSStdDev = sqrt(NominalDeltaT) * Eigen::Vector3d(errorBudget.XAccelVRWMPS2PerRtHzStdDev, errorBudget.YAccelVRWMPS2PerRtHzStdDev, errorBudget.ZAccelVRWMPS2PerRtHzStdDev);

    // Gyro Error Terms:
    // The parameters below are for the gyro in-run bias error
    if (InRunBiasOption == false)
    {
        GyroBiasPhi.x() = exp(-NominalDeltaT / errorBudget.XGyroAxisInRunBiasTimeConstantSec);
        GyroBiasPhi.y() = exp(-NominalDeltaT / errorBudget.YGyroAxisInRunBiasTimeConstantSec);
        GyroBiasPhi.z() = exp(-NominalDeltaT / errorBudget.ZGyroAxisInRunBiasTimeConstantSec);

        GyroBiasInRunStdDev.x() = errorBudget.XGyroAxisInRunBiasRadPerSecStdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.XGyroAxisInRunBiasTimeConstantSec));
        GyroBiasInRunStdDev.y() = errorBudget.YGyroAxisInRunBiasRadPerSecStdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.YGyroAxisInRunBiasTimeConstantSec));
        GyroBiasInRunStdDev.z() = errorBudget.ZGyroAxisInRunBiasRadPerSecStdDev * sqrt(1 - exp(-2 * NominalDeltaT / errorBudget.ZGyroAxisInRunBiasTimeConstantSec));
        // do initial draw for in-run bias
        //                GyroBiasInRun = diag([errorBudget.XGyroAxisInRunBiasRadPerSecStdDev;errorBudget.YGyroAxisInRunBiasRadPerSecStdDev;errorBudget.ZGyroAxisInRunBiasRadPerSecStdDev])*randn(GyroBiasInRunRandomNumberStreamPtr,3,1);
        GyroBiasInRun = Eigen::Vector3d(errorBudget.XGyroAxisInRunBiasRadPerSecStdDev, errorBudget.YGyroAxisInRunBiasRadPerSecStdDev, errorBudget.ZGyroAxisInRunBiasRadPerSecStdDev).asDiagonal() * RandomVector(GyroBiasDist, GyroBiasRNG);
    }
    // Following is the standard deviation of the angular random walk noise
    GyroARWRadStdDev = sqrt(NominalDeltaT) * Eigen::Vector3d(errorBudget.XGyroARWRadPerSecPerRtHzStdDev, errorBudget.YGyroARWRadPerSecPerRtHzStdDev, errorBudget.ZGyroARWRadPerSecPerRtHzStdDev);
}

NoisyIMUData IMUErrorSimulator::EachCycle(IMUErrorBudgetTemplate errorBudget, Eigen::Vector3d InputDeltaVBodyWRTInertialInBody, Eigen::Vector3d InputDeltaThetaBodyWRTInertialInBody) {
    Eigen::Vector3d quantizedAccelOutput, OutputBodyFrameAccelErrors, quantizedGyroOutput, OutputBodyFrameRateGyroErrors;
    NoisyIMUData returnValue;

    if (!InRunBiasOption)
    {
        TotalAccelBias = AccelBiasTurnOn + AccelBiasInRun;
        AccelBiasInRun = AccelBiasPhi.array() * AccelBiasInRun.array() + AccelBiasInRunStdDev.array() * RandomVector(AccelBiasDist, AccelBiasRNG).array();
    }

    AccelVRWMSDraws = AccelVRWMSStdDev.array() * RandomVector(AccelVRWDist, AccelVRWRNG).array();

    // Calculate errors to be added to the accel data
    OutputBodyFrameAccelErrors = TotalAccelBias + errorBudget.SampleFrequency * AccelVRWMSDraws + errorBudget.SampleFrequency * Ma * InputDeltaVBodyWRTInertialInBody;

    // Add errors to accel data
    Eigen::Vector3d OutputNoisyDeltaVBodyWRTInertialInBody = InputDeltaVBodyWRTInertialInBody + NominalDeltaT * OutputBodyFrameAccelErrors;

    // Take the noisy delta-V's calculated above in double precision
    // and quantize them based on the accel spec.
    quantizedAccelOutput = errorBudget.AccelDeltaVQuantizationMPS * ((OutputNoisyDeltaVBodyWRTInertialInBody + AccelQuantizationResiduals).array() / errorBudget.AccelDeltaVQuantizationMPS).round();
    // Calculate residuals from quantization process and later add them to
    // the next accel measurements to be quantized
    AccelQuantizationResiduals = OutputNoisyDeltaVBodyWRTInertialInBody + AccelQuantizationResiduals - quantizedAccelOutput;

    // Calculate totality of errors added to accel data:
    OutputBodyFrameAccelErrors = errorBudget.SampleFrequency * (quantizedAccelOutput - InputDeltaVBodyWRTInertialInBody).array();

    // Gyro
    // Calculate the total gyro bias, i.e. turn-on + in-run
    if (InRunBiasOption == false)
    {
        TotalGyroBias = GyroBiasTurnOn + GyroBiasInRun;
        // create the next in-run gyro bias error
        GyroBiasInRun = GyroBiasPhi.array() * GyroBiasInRun.array() + GyroBiasInRunStdDev.array() * RandomVector(GyroBiasDist, GyroBiasRNG).array();
    }
    // do random draw for gyro angel random walk:
    GyroARWRadDraws = GyroARWRadStdDev.array() * RandomVector(GyroARWDist, GyroARWRNG).array();

    // Calculate errors to be added to the gyro data
    OutputBodyFrameRateGyroErrors = TotalGyroBias.array() + errorBudget.SampleFrequency * GyroARWRadDraws.array() + errorBudget.SampleFrequency * (Mg * InputDeltaThetaBodyWRTInertialInBody).array();

    // Add errors to gyro data
    Eigen::Vector3d OutputNoisyDeltaThetaBodyWRTInertialInBody = InputDeltaThetaBodyWRTInertialInBody.array() + NominalDeltaT * OutputBodyFrameRateGyroErrors.array();

    // Take the noisy delta-theta's calculated above in double precision
    // and quantize them based on the gyro spec.
    quantizedGyroOutput = errorBudget.GyroDeltaThetaQuantizationRadians * ((OutputNoisyDeltaThetaBodyWRTInertialInBody + GyroQuantizationResiduals).array() / errorBudget.GyroDeltaThetaQuantizationRadians).round();
    // Calculate residuals from quantization process and later add them to
    // the next gyro measurements to be quantized
    GyroQuantizationResiduals = OutputNoisyDeltaThetaBodyWRTInertialInBody + GyroQuantizationResiduals - quantizedGyroOutput;

    // Calculate totality of errors added to gyro data:
    OutputBodyFrameRateGyroErrors = errorBudget.SampleFrequency * (quantizedGyroOutput - InputDeltaThetaBodyWRTInertialInBody).array();

    returnValue.noisyDeltaV = quantizedAccelOutput;
    returnValue.noisyDeltaTheta = quantizedGyroOutput;
    returnValue.deltaVErrors = OutputBodyFrameAccelErrors;
    returnValue.deltaThetaErrors = OutputBodyFrameRateGyroErrors;

    return returnValue;
}
