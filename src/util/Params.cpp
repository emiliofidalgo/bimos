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

#include "bimos/util/Params.h"

namespace bimos
{

Params* Params::_instance = 0;

/**
 * @brief Method for getting access to the singleton.
 * @return A reference to the unique object of the class.
 */
Params* Params::getInstance()
{
    if (_instance == 0)
    {
        _instance = new Params;
    }
    return _instance;
}

/**
 * @brief Updates the parameters using the info obtained from the node handle.
 * @param nh The node handle.
 */
void Params::readParams(const ros::NodeHandle& nh)
{
    nh.param<std::string>("working_dir", working_dir, "");
    ROS_INFO("[Params] Working directory: %s", working_dir.c_str());

    nh.param<std::string>("img_descriptor", img_descriptor, "FAST_LDB");
    ROS_INFO("[Params] Image description %s", img_descriptor.c_str());

    nh.param("nkeypoints", nkeypoints, 3000);
    ROS_INFO("[Params] Number of features: %i", nkeypoints);

    nh.param("pub_debug_info", pub_debug_info, false);
    ROS_INFO("[Params] Publish debug info: %i", pub_debug_info ? 1 : 0);

    nh.param("lc_delay_kfs", lc_delay_kfs, 5);
    ROS_INFO("[Params] LC delay buffer size: %i", lc_delay_kfs);

    nh.param("match_ratio", match_ratio, 0.8);
    ROS_INFO("[Params] NNDR: %f", match_ratio);

    nh.param("min_inliers", min_inliers, 200);
    ROS_INFO("[Params] Minimum number of inliers for LC: %i", min_inliers);

    nh.param("optim_every_kfs", optim_every_kfs, 15);
    ROS_INFO("[Params] Optimizer after every KFs: %i", optim_every_kfs);

    nh.param("blend_exp", blend_exp, false);
    ROS_INFO("[Params] Exposure compensator in blending %i", blend_exp ? 1 : 0);

    nh.param("kf_min_inliers", kf_min_inliers, 550);
    ROS_INFO("[Params] Minimum number of inliers for KF: %i", kf_min_inliers);

    nh.param("kf_overlap", kf_overlap, 0.4);
    ROS_INFO("[Params] Minimum overlap for KF: %f", kf_overlap);
}

}
