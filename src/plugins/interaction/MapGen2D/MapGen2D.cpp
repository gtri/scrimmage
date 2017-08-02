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

    //double resolution = sc::get<double>("resolution", map_parse.params(), 1.0);
    
    std::string filename = map_parse.params()["XML_DIR"] + "/" +
        map_parse.params()["filename"];
    
    cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    if (!img.data) {
        cout << "Failed to open file: " << filename << endl;
        return false;
    }
    cv::imshow("Original", img);

    // cv::Mat dst, cdst;
    // cv::Canny(img, dst, 50, 200, 3);
    // cv::cvtColor(dst, cdst, CV_GRAY2BGR);
    // 
    // std::vector<cv::Vec4i> lines;
    // cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    // for(size_t i = 0; i < lines.size(); i++ ) {
    //     cv::Vec4i l = lines[i];
    //     cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
    //               cv::Scalar(0,0,255), 3, CV_AA);
    // 
    //     cout << "Line: " << l << endl;
    // }
    // 
    // cv::imshow("lines", cdst);
    // cv::waitKey(10);
    
    /// Convert image to gray and blur it
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    cv::blur(gray, gray, cv::Size(3,3));
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    
    /// Detect edges using canny
    double thresh = 100;
    cv::Mat canny_output;
    cv::Canny(gray, canny_output, thresh, thresh*2, 3);
    /// Find contours
    cv::findContours(canny_output, contours, hierarchy, CV_CHAIN_APPROX_SIMPLE,
                     CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    
    /// Draw contours and publish 3D shapes for each cont
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    for(unsigned int i = 0; i < contours.size(); i++ ) {
        cv::drawContours(drawing, contours, i, cv::Scalar(0,0,255), 2, 8,
                         hierarchy, 0, cv::Point());
    
        //cout << "Point: " << contours[i] << endl;
        for (unsigned int j = 0; j < contours[i].size(); j++) {
            cv::circle(drawing, contours[i][j], 1, cv::Scalar(255,0,0), 1, 8, 0);
        }
    }    
    
    /// Show in a window
    cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cv::imshow( "Contours", drawing );
    
    cv::waitKey(10);
    
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
