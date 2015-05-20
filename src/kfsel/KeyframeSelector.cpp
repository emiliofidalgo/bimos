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

#include <bimos/kfsel/KeyframeSelector.h>

namespace bimos
{

/**
 * @brief Default constructor.
 */
KeyframeSelector::KeyframeSelector(const ros::NodeHandle& nh) :
    _nh(nh)
{
    _img_subs = _nh.subscribe("image", 1, &KeyframeSelector::processImage, this);
}

/**
 * @brief Default destructor.
 */
KeyframeSelector::~KeyframeSelector()
{
}

/**
 * @brief Entry point for running this class as a thread.
 */
void KeyframeSelector::run()
{
    ros::spin();
}

/**
 * @brief Process each image that arrives to the mosaicing algorithm.
 * @param msg Image as a ROS message.
 */
void KeyframeSelector::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("Image", cv_ptr->image);
    cv::waitKey(50);
}

}
