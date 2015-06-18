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

#include <cstdlib>
#include <fstream>
#include <signal.h>

#include <boost/thread.hpp>
#include <std_srvs/Empty.h>

#include <bimos/graph/MosaicGraph.h>
#include <bimos/imgdesc/ImageDescriptor.h>
#include <bimos/kfsel/KeyframeSelector.h>
#include <bimos/loopcloser/LoopCloser.h>
#include <bimos/optim/Optimizer.h>
#include <bimos/util/MosaicPublisher.h>
#include <bimos/util/Image.h>
#include <bimos/util/Params.h>
#include <bimos/util/util.h>

// Creating the MosaicGraph structure
bimos::MosaicGraph mgraph;

// Callback for optimize service
bool optimize(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[optim] Optimize the positions of the mosaic...");

    ceres::Solver::Summary summ;
    mgraph.optimize(summ);

    ROS_INFO("[optim] %s", summ.BriefReport().c_str());
    return true;
}

// Callback for blend service
bool blend(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("Blend!");
    return true;
}

int main(int argc, char** argv)
{    
    ROS_INFO("Initializing node ...");

    // ROS
    ros::init(argc, argv, "bimos");
    ros::NodeHandle nh("~");
    ros::ServiceServer optservice = nh.advertiseService("optimize", optimize);
    ros::ServiceServer blendservice = nh.advertiseService("blend", blend);

    bimos::Params* p = bimos::Params::getInstance();
    ROS_INFO("Reading parameters ...");
    p->readParams(nh);
    ROS_INFO("Parameters read");

    // Preparing working directory
    ROS_INFO("Preparing working directory ...");
    boost::filesystem::path res_imgs_dir = p->working_dir + "images/";
    boost::filesystem::remove_all(res_imgs_dir);
    boost::filesystem::create_directory(res_imgs_dir);
    res_imgs_dir = p->working_dir + "inliers/";
    boost::filesystem::remove_all(res_imgs_dir);
    boost::filesystem::create_directory(res_imgs_dir);
    ROS_INFO("Working directory ready");    

    // Creating the information publisher class
    bimos::MosaicPublisher mpublisher(nh, &mgraph, p);

    ROS_INFO("Node initialized");

    // Keyframe Selector Thread
    bimos::KeyframeSelector kfsel(nh, p, &mgraph);
    boost::thread kfsel_thread(&bimos::KeyframeSelector::run, &kfsel);

    // Loop Closer Thread
    bimos::LoopCloser lcloser(nh, p, &mgraph);
    boost::thread lcloser_thread(&bimos::LoopCloser::run, &lcloser);

    // Optimizer Thread
    bimos::Optimizer optim(p, &mgraph);
    boost::thread optim_thread(&bimos::Optimizer::run, &optim);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        if (p->pub_debug_info)
        {
            mpublisher.publishGraphInfo();
        }

        ros::spinOnce();
        rate.sleep();
    }

    optim_thread.join();
    lcloser_thread.join();
    kfsel_thread.join();

    return 0;
}
