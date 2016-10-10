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

#include "ros/ros.h"

#include <camera_calibration_parsers/parse_yml.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <bimos/util/util.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");

    // ROS Communication
    ros::NodeHandle nh("~");

    // Reading parameters
    std::string directory;
    nh.param<std::string>("images_dir", directory, "");
    ROS_INFO("[Param] Directory: %s", directory.c_str());

    std::string camera_info_file;
    nh.param<std::string>("camera_info_file", camera_info_file, "");
    ROS_INFO("[Param] Camera Info File: %s", camera_info_file.c_str());

    double frequency;
    nh.param("freq", frequency, 10.0);
    ROS_INFO("[Param] Frequency: %f", frequency);

    bool publish_ci = false;
    sensor_msgs::CameraInfo cinfo;
    std::string camera_name = "/camera";
    if (camera_info_file != "")
    {
        publish_ci = true;
        // Parsing calibration file
        camera_calibration_parsers::readCalibrationYml(camera_info_file, camera_name, cinfo);
    }

    ROS_INFO("Reading Images ...");
    std::vector<std::string> filenames;
    bimos::getImageFilenames(directory, filenames);
    ROS_INFO("Found %lu images", filenames.size());

    // Defining publishers
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_im = it.advertise("image_raw", 300);
    ros::Publisher pub_ci = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    // Iterating for publishing the images
    ros::Rate loop_rate(frequency);
    for (unsigned i = 0; i < filenames.size(); i++)
    {
        if (ros::isShuttingDown())
        {
            break;
        }

        ROS_INFO("Publishing image %u", i);

        // Preparing the message
        ros::Time now = ros::Time::now();
        cv::Mat image = cv::imread(filenames[i]);

        // Creating Image msg
        cv_bridge::CvImage cvb_image;
        cvb_image.image = image;
        cvb_image.encoding = image.channels() > 1 ? "bgr8" : "mono8";

        sensor_msgs::Image image_msg;
        cvb_image.toImageMsg(image_msg);
        image_msg.header.stamp = now;
        image_msg.header.frame_id = camera_name;
        pub_im.publish(image_msg);

        if (publish_ci)
        {
            cinfo.header.stamp = now;
            cinfo.header.frame_id = camera_name;
            pub_ci.publish(cinfo);
        }
        loop_rate.sleep();
	}

    ROS_INFO("All images have been published");

	return 0;
}
