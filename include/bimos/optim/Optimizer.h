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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>

#include <bimos/graph/MosaicGraph.h>
#include <bimos/util/Params.h>

namespace bimos
{

/**
 * @brief This class is executed as a thread every n inserted KFs for optimizing their positions
 */
class Optimizer
{
public:
    Optimizer(Params* params, MosaicGraph* _mgraph);
    ~Optimizer();

    void run();

private:

    // Parameters
    Params* p;

    // Graph Management
    MosaicGraph* mgraph;
};

}
#endif // OPTIMIZER_H
