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
KeyframeSelector::KeyframeSelector(const ros::NodeHandle& nh, Params* params) :
    _nh(nh),
    p(params)
{
    imgdesc = new ImageDescriptor(p->img_descriptor, p->nkeypoints);
}

/**
 * @brief Default destructor.
 */
KeyframeSelector::~KeyframeSelector()
{
    delete imgdesc;
}

/**
 * @brief Entry point for running this class as a thread.
 */
void KeyframeSelector::run()
{
    _img_subs = _nh.subscribe("image", 1, &KeyframeSelector::processImage, this);
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

    std::vector<cv::KeyPoint> kps;
    cv::Mat dscs;
    imgdesc->describeImage(cv_ptr->image, kps, dscs);
    std::cout << "KPS: " << kps.size() << std::endl;
    cv::Mat outimg;
    cv::drawKeypoints(img, kps, outimg);
    cv::imshow("KPS", outimg);
    cv::waitKey(50);
}

}
