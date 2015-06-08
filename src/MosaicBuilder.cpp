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
      p(Params::getInstance())
{
    ROS_INFO("Initializing node ...");
    ROS_INFO("Reading parameters ...");
    p->readParams(nh);
    ROS_INFO("Parameters read");
    ROS_INFO("Node initialized");

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

    ros::Rate rate(1.0);
    while (ros::ok())
    {
        // TODO Publishing Mosaicing Information
        rate.sleep();
    }

    ros::shutdown();    
}

}
