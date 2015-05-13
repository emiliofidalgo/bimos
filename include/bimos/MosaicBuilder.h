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
*/

#ifndef MOSAICBUILDER_H
#define MOSAICBUILDER_H

#include <omp.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace bimos
{

class MosaicBuilder
{
public:
    MosaicBuilder(const ros::NodeHandle nh);
    ~MosaicBuilder();

    // ROS
    ros::NodeHandle _nh;
};

}
#endif // MOSAICBUILDER_H
