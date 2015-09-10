/**
* This file is part of bimos.
*
* Copyright (C) 2015 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
* For more information see: <http://dmi.uib.es/~egarcia>
*
* bimos is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* bimos is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with bimos. If not, see <http://www.gnu.org/licenses/>.
*
* This code is based on the code of stitching_detailed.cpp in OpenCV.
*/

#include <cstdlib>
#include <fstream>
#include <omp.h>
#include <signal.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{    
    ROS_INFO("Conversion Factor Node");

	// ROS
    ros::init(argc, argv, "bimos_convfactors");
	ros::NodeHandle nh("~");

    // Reading parameters
    std::string working_dir;
    nh.param<std::string>("working_dir", working_dir, "");
    ROS_INFO("[Params] Working directory: %s", working_dir.c_str());

    double fx;
    nh.param("fx", fx, 1.0);
    ROS_INFO("[Params] Focal Length X: %f", fx);

    double fy;
    nh.param("fy", fy, 1.0);
    ROS_INFO("[Params] Focal Length Y: %f", fy);

    std::string distances_file;
    nh.param<std::string>("distances_file", distances_file, "");
    ROS_INFO("[Params] Distance file: %s", distances_file.c_str());

    std::ifstream ifile(distances_file.c_str(), std::ios::in);

    // Check to see that the file was opened correctly:
    if (!ifile.is_open())
    {
        std::cerr << "Unable to open the distances file!" << std::endl;
        exit(1);
    }

    // Since the Mosaic Frame is the first image, we get the first value as the distance to the ground
    double Z = 0.0;
    ifile >> Z;

    double convx = Z / fx;
    double convy = Z / fy;

    ROS_INFO("[MosaicBuilder] Conversion Factor in X coordinate: %f", convx);
    ROS_INFO("[MosaicBuilder] Conversion Factor in Y coordinate: %f", convy);

    cv::FileStorage fs(working_dir + "mosaic_convfactors.yml", cv::FileStorage::WRITE);

    fs << "convx" << convx;
    fs << "convy" << convy;
    fs << "dist_mframe" << Z;
    fs << "fx" << fx;
    fs << "fy" << fy;

    fs.release();

	return 0;
}
