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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_MATH_ANGLES_H_
#define INCLUDE_SCRIMMAGE_MATH_ANGLES_H_

namespace scrimmage {
class Angles {
 public:
    Angles();

    enum class Type {GPS, EUCLIDEAN};
    Angles(double angle, Type input_type, Type output_type);

    enum class Rotate {
        CCW = 0,
        CW
    };

///           Pos_Y
///             |
///             |
///   Neg_X --- 0 --- Pos_X
///             |
///             |
///           Neg_Y

    enum class HeadingZero {
        Pos_X = 1,
        Pos_Y = 2,
        Neg_X = 3,
        Neg_Y = 4
    };

    // Degrees to/from radians conversion
    static double deg2rad(double deg);
    static double rad2deg(double rad);

    // Seup the input coordinate system's clock direction and zero axis
    void set_input_clock_direction(Rotate direction);

    void set_input_zero_axis(HeadingZero zero);

    // Setup the input coordinate system's clock direction and zero axis
    void set_output_clock_direction(Rotate direction);

    void set_output_zero_axis(HeadingZero zero);

    // Is the provided angle between 0 and 359 degrees, inclusive?
    bool is_angle_360(double angle);

    // Normalize angle to 0-359 degrees
    static double angle_360(double angle);

    static double angle_2pi(double angle);

    // Normalize angle to 0-179 degrees
    static double angle_180(double angle);

    static double angle_pi(double angle);

    /*! \brief shortest angle from ang2 to ang1 */
    static double angle_diff(double ang1, double ang2);

    /*! \brief shortest angle from ang2 to ang1 (radians) */
    static double angle_diff_rad(double ang1, double ang2);

    /*! \brief returns whether ang is within the wedge between ang1 and ang2
     * where the wedge is less than 180 degrees
     */
    static bool angle_within(double ang1, double ang2, double ang);

    /*! \brief returns whether ang is within the wedge between ang1 and ang2
     * where the wedge is less than PI radians
     */
    static bool angle_within_rad(double ang1, double ang2, double ang);

    static double angle_avg(double ang1, double ang2);

    static double angle_avg_rad(double ang1, double ang2);

    // Set the input coordinate system's angle that the user wants to convert
    void set_angle(double angle);

    // Get the output coordinate system's computed angle.
    double angle();

 protected:
    double angle_ = 0;

    Rotate in_direction_ = Rotate::CCW;
    HeadingZero in_zero_ = HeadingZero::Pos_X;

    Rotate out_direction_ = Rotate::CCW;
    HeadingZero out_zero_ = HeadingZero::Pos_X;

 private:
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_MATH_ANGLES_H_
