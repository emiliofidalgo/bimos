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

#include <bimos/loopcloser/LoopCloser.h>

namespace bimos
{

/**
 * @brief Default constructor.
 */
LoopCloser::LoopCloser(const ros::NodeHandle& nh, Params* params, MosaicGraph* _mgraph) :
    _nh(nh),
    p(params),
    mgraph(_mgraph)
{    
}

/**
 * @brief Default destructor.
 */
LoopCloser::~LoopCloser()
{    
}

/**
 * @brief Entry point for running this class as a thread.
 */
void LoopCloser::run()
{
    ros::Rate r(500);
    while(ros::ok())
    {
        Keyframe* newkf;
        mgraph->newKFs.wait_and_pop(newkf);

        ROS_INFO("LOOP CLOSER: Processing new KF %i", newkf->id);

        r.sleep();
    }
}

}
