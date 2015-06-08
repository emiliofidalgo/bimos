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

#ifndef KFSELECTOR_H
#define KFSELECTOR_H

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <bimos/imgdesc/ImageDescriptor.h>
#include <bimos/graph/MosaicGraph.h>
#include <bimos/motionest/HomographyEstimator.h>
#include <bimos/util/Params.h>
#include <bimos/util/Image.h>

namespace bimos
{

/**
 * @brief This class is executed in a thread in order to select the images to be part of the mosaic.
 */
class KeyframeSelector
{
public:
    KeyframeSelector(const ros::NodeHandle& nh, Params* params, MosaicGraph* _mgraph);
    ~KeyframeSelector();

    void run();
    void processImage(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle _nh;
    ros::Subscriber _img_subs;

    // Parameters
    Params* p;

    // Graph Management
    MosaicGraph* mgraph;

    // Image descriptor
    ImageDescriptor* imgdesc;

    // Counters
    int nimages;
    int nkfs;
};

}
#endif // KFSELECTOR_H
