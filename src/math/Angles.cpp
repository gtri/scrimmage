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

#include <scrimmage/math/Angles.h>

#include <cmath>

#include <boost/math/special_functions/sign.hpp>

namespace scrimmage {

Angles::Angles() {}

Angles::Angles(double angle, Type input_type, Type output_type) {

    if (input_type == Type::GPS) {
        in_zero_ = HeadingZero::Pos_Y;
        in_direction_ = Rotate::CW;
    } else if (input_type == Type::EUCLIDEAN) {
        in_zero_ = HeadingZero::Pos_X;
        in_direction_ = Rotate::CCW;
    }

    if (output_type == Type::GPS) {
        out_zero_ = HeadingZero::Pos_Y;
        out_direction_ = Rotate::CW;
    } else if (output_type == Type::EUCLIDEAN) {
        out_zero_ = HeadingZero::Pos_X;
        out_direction_ = Rotate::CCW;
    }

    set_angle(angle);
}

double Angles::deg2rad(double deg) {return deg * M_PI / 180.0;}

double Angles::rad2deg(double rad) {return rad * 180.0 / M_PI;}

void Angles::set_input_clock_direction(Angles::Rotate direction) {
    in_direction_ = direction;
}

void Angles::set_input_zero_axis(Angles::HeadingZero zero) {
    in_zero_ = zero;
}

void Angles::set_output_clock_direction(Angles::Rotate direction) {
    out_direction_ = direction;
}

void Angles::set_output_zero_axis(Angles::HeadingZero zero) {
    out_zero_ = zero;
}

bool Angles::is_angle_360(double angle) {
    if (angle >= 0 && angle < 360) {
        return true;
    } else {
        return false;
    }
}

double Angles::angle_360(double angle) {
    if (angle >= 360) {
        angle -= 360*floor(angle/360.0);
    }

    if (angle < 0) {
        angle += 360*ceil(std::abs(angle)/360.0);
    }
    return angle;
}

double Angles::angle_2pi(double angle) {
    double pi2 = 2 * M_PI;
    if (angle >= pi2) {
        angle -= pi2 * floor(angle / pi2);
    }

    if (angle < 0) {
        angle += pi2 * ceil(std::abs(angle)/ pi2);
    }

    return angle;
}

double Angles::angle_180(double angle) {
    angle = angle_360(angle);
    if (angle > 180) {
        angle -= 360.0;
    }
    return angle;
}

double Angles::angle_pi(double angle) {
    angle = angle_2pi(angle);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

double Angles::angle_diff(double ang1, double ang2) {
    double diff = angle_360(angle_360(ang1) - angle_360(ang2));
    if (diff > 180) {
        diff -= 360;
    }
    return diff;
}

double Angles::angle_diff_rad(double ang1, double ang2) {
    return angle_pi(angle_pi(ang1) - angle_pi(ang2));
}

double Angles::angle_avg(double ang1, double ang2) {
    return angle_180(ang2 + angle_diff(ang1, ang2) / 2.0);
}

double Angles::angle_avg_rad(double ang1, double ang2) {
    return angle_pi(ang2 + angle_diff_rad(ang1, ang2) / 2.0);
}

void Angles::set_angle(double angle) {
    // Norm the angle
    angle = angle_360(angle);

    if (in_direction_ == out_direction_) {
        // If the angle increase is in the same direction
        // Just add or subtract a specific offset
        int rots = static_cast<int>(in_zero_) - static_cast<int>(out_zero_);
        angle_ = angle_360(angle + rots*90);
    } else {
        // If the coordinates are in different CW / CCW frames

        // Determine number of rotations
        // The sign is different depending on if the in-direction is
        // CW or CCW
        int rots;
        if (in_direction_ == Rotate::CW) {
            rots = static_cast<int>(in_zero_) - static_cast<int>(out_zero_);
        } else {
            rots = static_cast<int>(out_zero_) - static_cast<int>(in_zero_);
        }

        // If the coordinates are different CW/CCW, but the zero is on
        // the same axis, need to set rotations to 4 to get (90*4 = 360)
        if (rots == 0) {
            rots = 4;
        }
        angle_ = angle_360(rots*90 - angle);
    }
}

double Angles::angle() {
    return angle_;
}

bool Angles::angle_within(double ang1, double ang2, double ang) {
    double diff = angle_diff(ang1, ang2);
    double diff1 = angle_diff(ang, ang2);
    return boost::math::sign(diff) == boost::math::sign(diff1) &&
        std::abs(diff) > std::abs(diff1);
}

bool Angles::angle_within_rad(double ang1, double ang2, double ang) {
    double diff = angle_diff_rad(ang1, ang2);
    double diff1 = angle_diff_rad(ang, ang2);
    return boost::math::sign(diff) == boost::math::sign(diff1) &&
        std::abs(diff) > std::abs(diff1);
}
} // namespace scrimmage
