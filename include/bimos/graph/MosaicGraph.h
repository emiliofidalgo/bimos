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

#ifndef MOSAICGRAPH_H
#define MOSAICGRAPH_H

#include <boost/thread.hpp>

#include <bimos/graph/Edge.h>
#include <bimos/graph/Graph.h>
#include <bimos/graph/Keyframe.h>

namespace bimos
{

class MosaicGraph
{
public:
    MosaicGraph();
    ~MosaicGraph();

    void addKeyframe(Image* img, const double weight, const cv::Mat& t);
    void linkKFs(const int a, const int b, const double weight, const cv::Mat& t);
    bool existsEdge(const int ori, const int dest);

protected:
    // Structures
    Graph graph;
    std::vector<Keyframe*> kfs;
    std::map<int, std::map<int, Edge*> > edges;

    // Last Keyframe inserted
    Keyframe* last_kf_inserted;
    Keyframe* mosaic_frame;

    // Mutex to control the access to the mosaic graph
    boost::mutex mutex_mgraph;
};

}

#endif // MOSAICGRAPH_H
