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

#ifndef _MOSAICPUB_H
#define _MOSAICPUB_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <omp.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>

#include <bimos/graph/MosaicGraph.h>
#include <bimos/util/Params.h>
#include <bimos/util/util.h>

namespace bimos
{

/**
 * @brief Main Publisher class
 */
class MosaicPublisher
{
public:
    MosaicPublisher(const ros::NodeHandle _nh, MosaicGraph* graph, Params* _p);

    // Functions for publishing information about the mosaicing process
    void publishGraphInfo();

private:

    // ROS
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_graph;

    // Parameters
    Params* p;
    MosaicGraph* mgraph;
};

}
#endif /* _MOSAICPUB_H */
