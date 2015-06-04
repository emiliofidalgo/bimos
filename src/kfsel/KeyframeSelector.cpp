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
KeyframeSelector::KeyframeSelector(const ros::NodeHandle& nh, Params* params, MosaicGraph* _mgraph) :
    _nh(nh),
    p(params),
    mgraph(_mgraph),
    nimages(0),
    nkfs(0)
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
    nimages = 0;
    nkfs = 0;

    // Launching the topic for receiving images
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

    // Serialize the image on disk
    std::string image_filename_orig = p->working_dir + "images/image%06d.jpg";
    char name_orig[500];
    sprintf(name_orig, image_filename_orig.c_str(), nimages);
    cv::imwrite(name_orig, cv_ptr->image);

    // Creating a instance of the bimos::Image class
    Image* image = new Image;

    // Filling the image structure.
    image->id = nimages;
    image->filename = std::string(name_orig);
    cv_ptr->image.copyTo(image->image);
    imgdesc->describeImage(image->image, image->kps, image->dscs);
    ROS_INFO("Found %lu keypoints in image %i", image->kps.size(), nimages);
    nimages++;

    delete image;
}

}
