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

#include <scrimmage/common/TerrainEvaluator.h>
#include <error.h>
#include <string>

namespace scrimmage{
namespace common {

/** Function to read/parse VTK file and generate a Matrix of just the x,y,z values contained within
 * Takes in "terrain" as a string
 * Returns a matrix as shown below:
 *    [x1 x2 x3 ... xn]
 *    [y1 y2 y3 ... yn]
 *    [z1 z2 z3 ... zn]
 */
void TerrainEvaluator::create_elevation_matrix(std::string elevation_vtk) const {
    //read in vtk
    // char* home = getenv("HOME");
    global_min_z = std::numeric_limits<float>::infinity();
    global_max_z = -std::numeric_limits<float>::infinity();
    // std::string home_dir(home);
    // std::string terrain = home_dir + "/vtk_files/" + elevation_vtk;
    std::ifstream vtk(elevation_vtk);
    if (!vtk) {
        // std::cerr << "Couldn't find vtk file!!!" << std::endl;
        throw std::runtime_error("Couldn't find vtk file " + elevation_vtk + " in /home/vtk_files/");
    } else {
        
        std::string line;

        //int rows=0, cols=3; // only looking for x,y,z ignore other polydata
        int lineNum = 0;
        std::vector<std::vector<float> > allLocations;

        //index through header and find number of points line
        while(lineNum != 5 && getline(vtk, line)){
            lineNum++;}
         // if (lineNum == 5){
         //     std::cout << line << std::endl;}

        //find # of digits, then extract the number of coordinates in vtk for dynamic allocation later
        size_t i = 0;
        for(;i < line.length(); i++) {
            if(isdigit(line[i])) {
                break;
            }
        }
        line = line.substr(i, line.length() - i );
        int numCoordinates = atoi(line.c_str());

        //std::cout << i << std::endl;
        //std::cout << "numCoordinates is: " << numCoordinates << std::endl;

        ///////////////////////////////////////FILL VECTOR OF VECTORS WITH ALL VTK LOCATIONS///////////////////////////////////////////
        lineNum++;
        
        //Index through all coordinate points in VTK file, parse x,y,z for each line into a vector, creating a Matrix of size (numCoordinates X 3)
        for (int i = 0; i < numCoordinates; ++i)
        {
            std::getline(vtk,line);
            std::vector<float>temp_xyz;

            float local_x = 0;
            float local_y = 0;
            float local_z = 0;
               
            std::istringstream ss(line);
            ss >> local_x;
            ss >> local_y;
            ss >> local_z;

            temp_xyz.push_back(local_x);
            temp_xyz.push_back(local_y);
            temp_xyz.push_back(local_z);
            if (local_z < global_min_z) {
                global_min_z = local_z;
            } else if (local_z > global_max_z) {
                global_max_z = local_z;
            }
            
            allLocations.push_back(temp_xyz);
            
            lineNum++;


        }
        //Transpose of all coordinates to simplify search later

        TerrainEvaluator::T_allLocations = transpose(allLocations);


        vtk.close();

        set = true;
        std::cout << "[TE] VTK z bounds [" << global_min_z << ", " << global_max_z << "]" << std::endl;
        //return T_allLocations;
    }
}

/** Function to determine if current position is above or below elevation
  * Takes in x,y,z positions (UTM coordinate system) conversion from LAT/LON most likely needed
  * Returns bool: true if below terrain resulting in collision
  */
bool TerrainEvaluator::is_collision(const float xpos, const float ypos, const float zpos) const {
    //search for Y, then X starting at found Y
    int Y_idx; int X_idx; int allLocations_width;

    search_y(TerrainEvaluator::T_allLocations[1], ypos, &Y_idx, &allLocations_width);
    search_x(TerrainEvaluator::T_allLocations[0], xpos, Y_idx, allLocations_width, &X_idx);


        if (TerrainEvaluator::T_allLocations[2][X_idx] > zpos) {
            // std::cout << "x,y input into is_collision: " << xpos << "," << ypos << std::endl;
            // std::cout << "Ground Truth: " << Tvec[0][X_idx] << "," << Tvec[1][X_idx] << "," << Tvec[2][X_idx] << std::endl;

            return true;
        } else {
            return false;
        }
}

float TerrainEvaluator::min_z(const float xpos, const float ypos) const {
    int Y_idx; int X_idx; int allLocations_width;

    search_y(TerrainEvaluator::T_allLocations[1], ypos, &Y_idx, &allLocations_width);
    search_x(TerrainEvaluator::T_allLocations[0], xpos, Y_idx, allLocations_width, &X_idx);

    return TerrainEvaluator::T_allLocations[2][X_idx];
}


/** Function for searching in the Y direction of VTK elevation data should be used in conjuction with and before "search_x"
  * Takes in vtk vector and current Y position
  * Returns closest Y index and width of vector
  */
void TerrainEvaluator::search_y(std::vector<float> const& vec, float positionY, int *y_index, int *vec_width) const {
    int width = std::upper_bound(vec.begin(),vec.end(),vec[0]) - vec.begin();
    int y_location = std::lower_bound(vec.begin(), vec.end(), positionY) - vec.begin();
    *y_index = y_location;
    *vec_width = width;
}


/** Function for searching in the X direction of VTK elevation data after Y has already been found
  * Takes in vtk vector, current X position, Y index, vector width
  * Returns closet X index
  */
void TerrainEvaluator::search_x(std::vector<float> const& vec, float positionX,int search_start,int vec_width, int *x_index) const {
    //std::vector<int>::iterator begin = vec[search_start];
    //std::cout << "search_start: " << search_start << std::endl;
    int x_location = std::lower_bound((vec.begin()+search_start), vec.begin()+search_start+vec_width, positionX) - vec.begin();

    *x_index = x_location;
}   


/** Function to transpose vtk vector for simplified searching using standard vector functions
  * Takes in vtk vector
  * Returns transposed vector
  */
std::vector<std::vector<float>> TerrainEvaluator::transpose(std::vector<std::vector<float>> &vec) const
{

    std::vector<std::vector<float> > t_vec(vec[0].size(), std::vector<float>());

    for (unsigned int i = 0; i < vec.size(); i++){
        for (unsigned int j = 0; j < vec[i].size(); j++){
            t_vec[j].push_back(vec[i][j]);
        }
    }
    return t_vec;
}

/* Used for testing purposes only
 * Takes in vtk vector
 * Returns random X,Y,Z tuple
 */
std::tuple<float, float, float> TerrainEvaluator::test_point(std::vector<std::vector<float> > &vec) const
{
    int random = rand() % vec[1].size();
    float randomX = vec[0][random];
    //std::cout << "random: " << random << std::endl;
    float randomY = vec[1][random];
    float randomZ = vec[2][random];
    //float randomY = 0;
    //float randomZ = 0;
    return std::make_tuple(randomX,randomY,randomZ);
}

}   // namespace common
}   // namespace chasm_lib

///FIND OUT WHAT VERSION OF VTK SCRIMMAGE IS USING || VTK8.2
/// 
