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
 * Class for adding realistic error/noise to 'perfect' IMU data generated from position, velocity,
 * orientation data
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORSIMULATOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORSIMULATOR_H_

#include <scrimmage/plugins/sensor/ROSIMUSensor/IMUErrorBudgetTemplate.h>

#include <unsupported/Eigen/MatrixFunctions>

#include <random>

struct NoisyIMUData {
    Eigen::Vector3d noisyDeltaV;
    Eigen::Vector3d noisyDeltaTheta;
    Eigen::Vector3d deltaVErrors;
    Eigen::Vector3d deltaThetaErrors;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IMUErrorSimulator {
 public:
    bool InRunBiasOption;  // false = 1st order Gauss-Markov process, true = Barnes-Jarvis model
    double InitRandomNumberSeed = 1;
    int RandomNumberSeedsPerInstance = 20;
    double NominalDeltaT;
    Eigen::Matrix3d Ma;
    Eigen::Vector3d AccelScaleFactorErr;
    Eigen::Vector3d AccelBiasTurnOn;
    Eigen::Vector3d AccelBiasInRun;
    Eigen::Vector3d AccelBiasPhi;
    Eigen::Vector3d AccelBiasInRunStdDev;
    Eigen::Vector3d TotalAccelBias;
    Eigen::Vector3d AccelVRWMSStdDev;
    Eigen::Vector3d AccelVRWMSDraws;
    Eigen::Vector3d AccelQuantizationResiduals = Eigen::Vector3d(0, 0, 0);

    Eigen::Matrix3d Mg;
    Eigen::Vector3d GyroScaleFactorErr;
    Eigen::Vector3d GyroBiasTurnOn;
    Eigen::Vector3d GyroBiasInRun;
    Eigen::Vector3d GyroBiasPhi;
    Eigen::Vector3d GyroBiasInRunStdDev;
    Eigen::Vector3d TotalGyroBias;
    Eigen::Vector3d GyroARWRadStdDev;
    Eigen::Vector3d GyroARWRadDraws;
    Eigen::Vector3d GyroQuantizationResiduals;

    // I did some testing with these, it seems that generation is only repeatable with separate
    // generator/distribution pairs. If you use multiple distributions on the same generator or the
    // same distribution on multiple generators the results are not the same as a unique pair.
    std::mt19937 InitRNG = std::mt19937(1);  // pass the seed
    std::mt19937 AccelBiasRNG = std::mt19937(2);
    std::mt19937 AccelVRWRNG = std::mt19937(3);
    std::mt19937 GyroBiasRNG = std::mt19937(4);
    std::mt19937 GyroARWRNG = std::mt19937(5);

    std::normal_distribution<double> InitDist =
        std::normal_distribution<double>(0, 1);  // mean 0, std deviation 1
    std::normal_distribution<double> AccelBiasDist =
        std::normal_distribution<double>(0, 1);  // mean 0, std deviation 1
    std::normal_distribution<double> AccelVRWDist =
        std::normal_distribution<double>(0, 1);  // mean 0, std deviation 1
    std::normal_distribution<double> GyroBiasDist =
        std::normal_distribution<double>(0, 1);  // mean 0, std deviation 1
    std::normal_distribution<double> GyroARWDist =
        std::normal_distribution<double>(0, 1);  // mean 0, std deviation 1

    explicit IMUErrorSimulator(IMUErrorBudgetTemplate& errorBudget);

    // generate a 3d vector with the passed in random distribution and generator
    Eigen::Vector3d RandomVector(std::normal_distribution<double>& distribution,
                                 std::mt19937& generator);

    void PerformInitialRandomDraws(IMUErrorBudgetTemplate& errorBudget);

    void CalculateParametersForFixedDeltaT(IMUErrorBudgetTemplate& errorBudget);

    NoisyIMUData EachCycle(IMUErrorBudgetTemplate& errorBudget,
                           Eigen::Vector3d InputDeltaVBodyWRTInertialInBody,
                           Eigen::Vector3d InputDeltaThetaBodyWRTInertialInBody);

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_IMUERRORSIMULATOR_H_
