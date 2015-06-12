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

#include "bimos/MosaicBuilder.h"

namespace bimos
{

/**
 * @brief Default class constructor.
 * @param nh ROS node handle.
 */
MosaicBuilder::MosaicBuilder(const ros::NodeHandle _nh)
    : nh(_nh),
      it(_nh),
      p(Params::getInstance())
{
    ROS_INFO("Initializing node ...");
    ROS_INFO("Reading parameters ...");
    p->readParams(nh);
    ROS_INFO("Parameters read");
    ROS_INFO("Node initialized");

    // Starting ROS publishers
    pub_graph = it.advertise("mosaic_graph", 1);

    createMosaic();
}

/**
 * @brief Default class destructor.
 */
MosaicBuilder::~MosaicBuilder()
{    
}

/**
 * @brief Starts the mosaicing process using the options stored in params.
 */
void MosaicBuilder::createMosaic()
{
    // Preparing working directory
    ROS_INFO("Preparing working directory ...");
    boost::filesystem::path res_imgs_dir = p->working_dir + "images/";
    boost::filesystem::remove_all(res_imgs_dir);
    boost::filesystem::create_directory(res_imgs_dir);
    ROS_INFO("Working directory ready");

    // Creating the MosaicGraph structure
    MosaicGraph mgraph;

    // Keyframe Selector Thread
    KeyframeSelector kfsel(nh, p, &mgraph);
    boost::thread kfsel_thread(&KeyframeSelector::run, &kfsel);

    // Loop Closer Thread
    LoopCloser lcloser(nh, p, &mgraph);
    boost::thread lcloser_thread(&LoopCloser::run, &lcloser);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        if (p->pub_debug_info)
        {
            publishGraphInfo(&mgraph);
        }

        rate.sleep();
    }

    kfsel_thread.join();
    lcloser_thread.join();    

    ros::shutdown();
}

/**
 * @brief This function creates a graph according to the graph information an publish it as an image
 */
void MosaicBuilder::publishGraphInfo(MosaicGraph* mgraph)
{
    // Getting graph information
    std::string dot_graph;
    mgraph->getDotGraph(dot_graph);

    // Storing the .dot file
    std::string mgraph_file = p->working_dir + "mgraph.dot";
    std::ofstream out(mgraph_file.c_str());
    out << dot_graph;
    out.close();

    // Calling to the conversion function
    std::string command = "dot -Kneato -n -Tpng " + p->working_dir + "mgraph.dot -o " + p->working_dir + "mgraph.png";
    std::system(command.c_str());

    // Publishing the image
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "mosaic_graph";
    cv::Mat img_graph = cv::imread(p->working_dir + "mgraph.png");

    sensor_msgs::Image img_msg;
    cv_bridge::CvImage cv_image(hdr, "bgr8", img_graph);
    cv_image.toImageMsg(img_msg);
    pub_graph.publish(img_msg);
}

}
