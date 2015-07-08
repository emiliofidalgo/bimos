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

#include <bimos/optim/Optimizer.h>

namespace bimos
{

/**
 * @brief Default constructor.
 */
Optimizer::Optimizer(Params* params, MosaicGraph* _mgraph) :
    p(params),
    mgraph(_mgraph),
    last_kf_optim(0)
{
}

/**
 * @brief Default destructor.
 */
Optimizer::~Optimizer()
{
}

/**
 * @brief Entry point for running this class as a thread.
 */
void Optimizer::run()
{
    ros::Rate r(200);
    while(mgraph->isBuilding())
    {
        // Checking if we have to execute an optimization
        int nkfs = mgraph->getNumberOfKeyframes();
        if (nkfs - last_kf_optim > p->optim_every_kfs)
        {
            ROS_INFO("[optim] Optimize the positions of the mosaic...");

            ceres::Solver::Summary summ;
            mgraph->optimize(summ, true);

            ROS_INFO("[optim] %s", summ.BriefReport().c_str());

            // Updating the last KF optimized
            last_kf_optim = nkfs;
        }

        // Sleeping the needed time
        ros::spinOnce();
        r.sleep();
    }
}

}
