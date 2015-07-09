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
#include <omp.h>
#include <signal.h>

#include <boost/thread.hpp>
#include <std_msgs/Empty.h>

#include <bimos/blend/Blender.h>
#include <bimos/graph/MosaicGraph.h>
#include <bimos/imgdesc/ImageDescriptor.h>
#include <bimos/kfsel/KeyframeSelector.h>
#include <bimos/loopcloser/LoopCloser.h>
#include <bimos/optim/Optimizer.h>
#include <bimos/util/MosaicPublisher.h>
#include <bimos/util/Image.h>
#include <bimos/util/Params.h>
#include <bimos/util/util.h>

// Parameters
bimos::Params* p;

// Creating the MosaicGraph structure
bimos::MosaicGraph* mgraph;

// Blender class
bimos::Blender blender;

// Write the final poses of the mosaic into a file
void writePoses()
{
    // Writing poses to a file
    ROS_INFO("Writing poses to a file ...");
    std::string filename = p->working_dir + "mosaic_poses.txt";
    std::ofstream file;
    file.open(filename.c_str());
    for (int kf_ind = 0; kf_ind < mgraph->getNumberOfKeyframes(); kf_ind++)
    {
        bimos::Keyframe* kf = mgraph->getKeyframe(kf_ind);

        std::vector<double> params;
        kf->trans.decomposeTransformation(params);

        file << params[0] << "\t" << params[1] << std::endl;
    }
    file.close();
    ROS_INFO("Pose file completed");
}

void createMosaic(ros::NodeHandle& nh)
{
    ROS_INFO("Initializing mosaicing process ...");

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
    bimos::MosaicPublisher mpublisher(nh, mgraph, p);

    ROS_INFO("Node initialized");

    // Keyframe Selector Thread
    bimos::KeyframeSelector kfsel(nh, p, mgraph);
    boost::thread kfsel_thread(&bimos::KeyframeSelector::run, &kfsel);

    // Loop Closer Thread
    bimos::LoopCloser lcloser(nh, p, mgraph);
    boost::thread lcloser_thread(&bimos::LoopCloser::run, &lcloser);

    // Optimizer Thread
    bimos::Optimizer optim(p, mgraph);
    boost::thread optim_thread(&bimos::Optimizer::run, &optim);

    // Assigning parameters to the blender
    blender.setParams(p);
    blender.setGraph(mgraph);

    ros::Rate rate(0.5);
    while (mgraph->isBuilding())
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

    ROS_INFO("Mosaic finished");
}

// Callback for optimize service
void optimize(const std_msgs::EmptyConstPtr& msg)
{
    ROS_INFO("[optim] Optimize the positions of the mosaic...");

    double init_time = omp_get_wtime();
    ceres::Solver::Summary summ;
    mgraph->optimize(summ, false);
    double end_time = omp_get_wtime();

    writePoses();

    ROS_INFO("[optim] %s", summ.FullReport().c_str());
    ROS_INFO("[optim] Optimization time: %f", end_time - init_time);
}

// Callback for blend service
void blend(const std_msgs::EmptyConstPtr& msg)
{
    boost::thread blender_thread(&bimos::Blender::run, &blender);
}

// Callback for optim_blend service
void optim_blend(const std_msgs::EmptyConstPtr& msg)
{
    ROS_INFO("[optim] Optimize the positions of the mosaic...");

    double init_opttime = omp_get_wtime();
    ceres::Solver::Summary summ;
    mgraph->optimize(summ, false);
    double end_opttime = omp_get_wtime();

    writePoses();

    ROS_INFO("[optim] %s", summ.FullReport().c_str());

    double init_blentime = omp_get_wtime();
    blender.run();
    double end_blentime = omp_get_wtime();

    double graph_time = mgraph->getMosaicTime();

    ROS_INFO("---");
    ROS_INFO("[Timing]");
    ROS_INFO("Alignment time: %f", graph_time);
    ROS_INFO("Optimization time: %f", end_opttime - init_opttime);
    ROS_INFO("Blending: %f", end_blentime - init_blentime);
    ROS_INFO("Total:: %f s", graph_time + (end_opttime - init_opttime) + (end_blentime - init_blentime));

    double avg, stddev;
    std::string inliers_dir = p->working_dir + "inliers/";
    mgraph->getMosaicError(avg, stddev, inliers_dir);

    ROS_INFO("---");
    ROS_INFO("[Quality Measurements]");
    ROS_INFO("Total number of images: %i", mgraph->getNumberOfImages());
    ROS_INFO("Selected keyframes: %i", mgraph->getNumberOfKeyframes());
    ROS_INFO("Average error in pixels: %f", avg);
    ROS_INFO("Stddev  error in pixels: %f", stddev);
    ROS_INFO("Successful observations: %i", mgraph->getObsOK());
    ROS_INFO("Unsuccessful observations: %i", mgraph->getObsNOK());
}

// Callback for init_mosaic service
void init_mosaic(const std_msgs::EmptyConstPtr& msg)
{
    if (mgraph)
    {
        delete mgraph;
    }

    mgraph = new bimos::MosaicGraph();
    mgraph->setBuildingState(true);
    ros::NodeHandle nh("~");

    createMosaic(nh);    
}

// Callback for stop_mosaic service
void stop_mosaic(const std_msgs::EmptyConstPtr& msg)
{
    ROS_INFO("Stopping mosaic process ...");

    if (mgraph)
    {
        mgraph->setBuildingState(false);
    }
}

// Main function
int main(int argc, char** argv)
{
    ROS_INFO("Initializing node ...");

    // ROS
    ros::init(argc, argv, "bimos");
    ros::NodeHandle nh("~");
    ros::Subscriber optservice = nh.subscribe("optimize", 1, optimize);
    ros::Subscriber blendservice = nh.subscribe("blend", 1, blend);
    ros::Subscriber optimblendservice = nh.subscribe("optim_and_blend", 1, optim_blend);
    ros::Subscriber initmosservice = nh.subscribe("init_mosaic", 1, init_mosaic);
    ros::Subscriber stopmosservice = nh.subscribe("stop_mosaic", 1, stop_mosaic);

    // Initializing mgraph structure
    mgraph = 0;

    // Reading parameters
    p = bimos::Params::getInstance();
    //ROS_INFO("Reading parameters ...");
    //p->readParams(nh);
    //ROS_INFO("Parameters read");

    ros::spin();
}
