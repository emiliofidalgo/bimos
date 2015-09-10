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

#ifndef LOOPCLOSER_H
#define LOOPCLOSER_H

#include <ros/ros.h>

#include <bimos/graph/MosaicGraph.h>
#include <bimos/util/Params.h>
#include <bimos/motionest/HomographyEstimator.h>
#include <obindex/BinaryIndex.h>

namespace vi = obindex;

namespace bimos
{

/**
 * @brief This class is executed in a thread in order to detect loop between new inserted keyframes and the existents in the graph.
 */
class LoopCloser
{
public:
    LoopCloser(const ros::NodeHandle& nh, Params* params, MosaicGraph* _mgraph);
    ~LoopCloser();

    void run();    

private:
    ros::NodeHandle _nh;    

    // Parameters
    Params* p;

    // Graph Management
    MosaicGraph* mgraph;

    // Delay buffer
    std::queue<Keyframe* > buffer;

    // Binary index
    vi::BinaryIndex* bindex;

    // Variable to manage BinaryIndex initialization
    bool bindex_init;
};

}
#endif // LOOPCLOSER_H
