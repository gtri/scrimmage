/**
 *****************************************************************************
 * @file
 * @section LICENSE
 *  ----------------------------------------------------------------------------
 *    ----------------------------- UNCLASSIFIED -----------------------------
 *  ----------------------------------------------------------------------------
 *
 *  Copyright 2016, Georgia Tech Research Corporation, Atlanta, Georgia 30332
 *
 *  LICENSE TO COPY: This material may be reproduced by or for the U.S. Government
 *  pursuant to the copyright license under the clauses at DFARS 252.227-7013 and
 *  252.227-7014.
 *
 *  DISTRIBUTION STATEMENT D: Distribution authorized to the Department of Defense
 *  and U.S. DoD contractors only (Critical Technical Data, April 28, 2014).
 *
 *  WARNING: This document contains technical data whose export is restricted by
 *  the Arms Export Control Act (Title 22, U.S.C., Sec 2751, et seq.) or the
 *  Export Administration Act of 1979, as amended (Title 50, U.S.C., App. 2401 et
 *  seq.). Violations of these export laws are subject to severe criminal
 *  penalties. Disseminate in accordance with provisions of DoD Directive 5230.25.
 *
 *  HANDLING AND DESTRUCTION NOTICE: Comply with the distribution statement and
 *  destroy this document by any method that will prevent disclosure or
 *  reconstruction of this documents contents.
 *  ----------------------------------------------------------------------------
 * @author Edward Stevens <edward.stevens@gtri.gatech.edu>
 * @date 1 April 2022
 * @version 0.1.0
 * @brief Takes in X,Y and returns Altitude
 * @section DESCRIPTION
 * Given X and Y location a VTK query can be completed to return Z/Altitude
 *
 */

#ifndef SCRIMMAGE_INCLUDE_COMMON_TERRAIN_EVALUATOR_H_
#define SCRIMMAGE_INCLUDE_COMMON_TERRAIN_EVALUATOR_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <tuple>
#include <exception>

#include <Eigen/Dense>
#include <omp.h>

namespace scrimmage {
namespace common {

class TerrainEvaluator {
private:
    // Parsed VTK for use within TerrainEvaluator
    inline static std::vector<std::vector<float> > T_allLocations;
    inline static bool set;
    int los_count = 0;

public:
    /** Function to determine if current position is above or below elevation
      * Takes in x,y,z positions (UTM coordinate system) conversion from LAT/LON most likely needed
      * Returns bool: true if below terrain resulting in collision
      */

    inline static int terrain_utm_zone;
    inline static float global_min_z;
    inline static float global_max_z;


    bool is_collision(const float xpos, const float ypos,const float zpos) const;

    bool line_of_sight(const Eigen::Vector3d &p1,
                       Eigen::Vector3d p2,
                       std::function<std::vector<double>(Eigen::Vector3d)> func,
                       int num_points) {
        ++los_count;
        Eigen::Vector3d dir = p2 - p1;
        double distance = dir.norm();
        double step_size = distance / num_points;
        dir = dir / distance;
        bool los = true;

        Eigen::Vector3d interp_pos = p1 + ((step_size) * dir);
        std::vector<double> new_coords = func(interp_pos);
        if (is_collision(new_coords[0], new_coords[1], new_coords[2])) {
            los = false;
        } else {
            {
                for (int i = 2; i < num_points; i++) {
                    Eigen::Vector3d interp_pos = p1 + ((i*step_size) * dir);
                    std::vector<double> new_coords = func(interp_pos);
                    if (is_collision(new_coords[0], new_coords[1], new_coords[2])) {
                        {
                            los = false;
                        }
                    }
                }
            }
        }
        return los;
    }


    float min_z(const float xpos, const float ypos) const;

    bool is_set() {
      return TerrainEvaluator::set;
    }


    /** Function to read/parse VTK file and generate a Matrix of just the x,y,z values contained within
     * Takes in "terrain" as a string
     * Returns a matrix as shown below:
     *    [x1 x2 x3 ... xn]
     *    [y1 y2 y3 ... yn]
     *    [z1 z2 z3 ... zn]
     */
    void create_elevation_matrix(std::string vtk) const;

    /** Function for searching in the Y direction of VTK elevation data should be used in conjuction with and before "search_x"
      * Takes in vtk vector and current Y position
      * Returns closest Y index and width of vector
      */
    void search_y(std::vector<float> const& vec, float positionY, int *y_index, int *vec_width) const;


    /** Function for searching in the X direction of VTK elevation data after Y has already been found
      * Takes in vtk vector, current X position, Y index, vector width
      * Returns closet X index
      */
    void search_x(std::vector<float> const& vec, float positionX, int search_start, int vec_width, int *x_index) const;


    /* Used for testing purposes only
     * Takes in vtk vector
     * Returns random X,Y,Z tuple
     */
    std::tuple<float, float, float> test_point(std::vector<std::vector<float> > &vec) const;


    /** Function to transpose vtk vector for simplified searching using standard vector functions
      * Takes in vtk vector
      * Returns transposed vector
      */
    std::vector<std::vector<float> > transpose(std::vector<std::vector<float> > &vec) const;
};

} // namespace common
} // namespace scrimmage

#endif //SCRIMMAGE_INCLUDE_COMMON_TERRAIN_EVALUATOR_H_
