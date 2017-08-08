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

#include <memory>
#include <limits>
#include <iostream>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/plugins/interaction/MapGen2D/MapGen2D.h>

#include <scrimmage/proto/ProtoConversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction, MapGen2D, MapGen2D_plugin)

MapGen2D::MapGen2D()
{     
}

bool MapGen2D::init(std::map<std::string,std::string> &mission_params,
                    std::map<std::string,std::string> &plugin_params)
{

    show_polygon_count_ = sc::get<bool>("show_polygon_count", plugin_params,
                                        false);
    
    ///////////////////////////////
    // Find the map params
    sc::ConfigParse map_parse;
    map_parse.set_required("filename");
    map_parse.set_required("resolution");
    
    sc::FileSearch file_search;
    std::map<std::string, std::string> overrides; // empty, no overrides
    if (!map_parse.parse(overrides, plugin_params["map"],
                            "SCRIMMAGE_DATA_PATH", file_search)) {        
        cout << "Failed to find map: " << plugin_params["map"] << endl;
        return false;
    }

    resolution_ = sc::get<double>("resolution", map_parse.params(), 1.0);
    wall_bottom_z_ = sc::get<double>("wall_bottom_z", map_parse.params(), 0.0);
    wall_height_ = sc::get<double>("wall_height", map_parse.params(), 5.0);
    wall_thickness_ = sc::get<double>("wall_thickness", map_parse.params(), 0.1);
    polygon_simplification_ = sc::get<double>("polygon_simplification", map_parse.params(), 3);    
    
    std::string filename = map_parse.params()["XML_DIR"] + "/" +
        map_parse.params()["filename"];
    
    cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    if (!img.data) {
        cout << "Failed to open file: " << filename << endl;
        return false;
    }
    cv::imshow("Original", img);

#if 0    
    cv::Mat dst, cdst;
    cv::Canny(img, dst, 50, 200, 3);
    cv::cvtColor(dst, cdst, CV_GRAY2BGR);    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, CV_PI/180, 3, 3, 10 );
    int line_count = 0;
    for(size_t i = 0; i < lines.size(); i++ ) {
        cv::Vec4i l = lines[i];
        cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                  cv::Scalar(0,0,255), 3, CV_AA);
    
        cv::circle(cdst, cv::Point(l[0], l[1]), 4, cv::Scalar(255,0,0), 1, 8, 0);
        cv::circle(cdst, cv::Point(l[2], l[3]), 4, cv::Scalar(255,0,0), 1, 8, 0);
    
        cout << "Line: " << l << endl;
        line_count++;
    }

    cout << "line count: " << line_count << endl;
    
    cv::imshow("lines", cdst);
    cv::waitKey(10);
#endif
    
    /// Convert image to gray and blur it
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    cv::blur(gray, gray, cv::Size(3,3));
    
    std::vector<std::vector<cv::Point>> contours_initial;
    std::vector<cv::Vec4i> hierarchy;
    
    /// Detect edges using canny
    double thresh = 100;
    cv::Mat canny_output;
    cv::Canny(gray, canny_output, thresh, thresh*2, 3);
    /// Find contours
    cv::findContours(canny_output, contours_initial, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_TC89_L1, cv::Point(0,0));    

    std::vector<std::vector<cv::Point> > contours;
    contours.resize(contours_initial.size());
    for (size_t k = 0; k < contours_initial.size(); k++) {
        cv::approxPolyDP(cv::Mat(contours_initial[k]), contours[k],
                         polygon_simplification_, false);
    }
    
    int count = 0;
    
    /// Draw contours and publish 3D shapes for each cont
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    for(unsigned int i = 0; i < contours.size(); i++ ) {
        cv::drawContours(drawing, contours, i, cv::Scalar(0,0,255), 2, 8,
                         hierarchy, 0, cv::Point());

        Eigen::Vector3d prev_p;
        for (unsigned int j = 0; j < contours[i].size(); j++) {
            cv::Point p_img = contours[i][j];

            Eigen::Vector3d p(p_img.x * resolution_,
                              (img.rows - p_img.y) * resolution_, 0);
            
            if (j > 0) {
                shapes_.push_back(connect_points(p, prev_p));
            }
            prev_p = p;
            
            cv::circle(drawing, p_img, 1, cv::Scalar(255,0,0), 1, 8,
                       0);
            count++;
        }
        Eigen::Vector3d p_first(contours[i][0].x * resolution_,
                                (img.rows - contours[i][0].y) * resolution_,
                                0);
        shapes_.push_back(connect_points(prev_p, p_first));        
    }                            

    if (show_polygon_count_) {
        cout << "Wall Polygon Count: " << count << endl;

        /// Show in a window
        cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        cv::imshow( "Contours", drawing );
        cv::waitKey(10);
    }
    
    return true;
}

bool MapGen2D::step_entity_interaction(std::list<sc::EntityPtr> &ents, 
                                                  double t, double dt)
{
    if (ents.empty())
    {
        return true;
    }

    return true;
}

std::shared_ptr<sp::Shape> MapGen2D::connect_points(Eigen::Vector3d &p,
                                                    Eigen::Vector3d &prev_p)
{
#if 0    
    double theta = atan2(p(1) - prev_p(1), p(0) - prev_p(0));
    sc::Quaternion quat(0,0,theta);    
    
    Eigen::Vector3d center((p(0) + prev_p(0)) / 2,
                           (p(1) + prev_p(1)) / 2,
                           wall_bottom_z_ + wall_height_ / 2.0);

    std::shared_ptr<sp::Shape> wall(new sp::Shape);
    wall->set_type(sp::Shape::Cube);
    sc::set(wall->mutable_color(), 0, 0, 255);
    wall->set_opacity(1.0);
    sc::set(wall->mutable_center(), center);
    sc::set(wall->mutable_xyz_lengths(), (p-prev_p).norm(),
            wall_thickness_, wall_height_);
    sc::set(wall->mutable_quat(), quat);
    
#else    
    std::shared_ptr<sp::Shape> wall(new sp::Shape);
    wall->set_type(sp::Shape::Polygon);
    sc::set(wall->mutable_color(), 0, 0, 255);
    wall->set_opacity(1.0);

    sc::add_point(wall, p + Eigen::Vector3d::UnitZ()*wall_bottom_z_);
    sc::add_point(wall, prev_p + Eigen::Vector3d::UnitZ()*wall_bottom_z_);
    sc::add_point(wall, prev_p + Eigen::Vector3d::UnitZ()*(wall_bottom_z_ + wall_height_));
    sc::add_point(wall, p + Eigen::Vector3d::UnitZ()*(wall_bottom_z_ + wall_height_));        
#endif
    wall->set_persistent(true);
    
    return wall;    
}
