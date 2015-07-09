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

#include <sstream>

#include <boost/thread.hpp>

#include <bimos/graph/Edge.h>
#include <bimos/graph/Graph.h>
#include <bimos/graph/Keyframe.h>
#include <bimos/optim/MosaicAdjuster.h>
#include <bimos/util/ConcurrentQueue.hpp>
#include <bimos/util/util.h>

namespace bimos
{

class MosaicGraph
{
public:
    MosaicGraph();
    ~MosaicGraph();

    int addKeyframe(Image* img, const double weight, const cv::Mat& t);
    void linkKFs(const int a, const int b, const double weight, const cv::Mat& t);
    void addConstraints(Keyframe* kf_prev, Keyframe* kf, std::vector<cv::DMatch>& matches);
    void optimize(ceres::Solver::Summary& summary, bool opt_local = true);
    bool existsEdge(const int ori, const int dest);

    Keyframe* getLastInsertedKF();
    Keyframe* getMosaicFrame();
    Keyframe* getKeyframe(const int id);
    int getNumberOfKeyframes();
    void getKFTransforms(std::vector<Transform>& transforms);
    void getDotGraph(std::string& contents);
    void getMosaicError(double& avg, double& stddev, std::string& inliers_dir);
    double getMosaicTime();
    void setBuildingState(bool value);
    bool isBuilding();

    void incrNumImages();
    int getNumberOfImages();
    void incrObsOK();
    int getObsOK();
    void incrObsNOK();
    int getObsNOK();

    // Queues for thread intercommunication
    ConcurrentQueue<Keyframe *> newKFs;

protected:
    // Structures
    Graph graph;
    std::vector<Keyframe*> kfs;
    std::map<int, std::map<int, Edge*> > edges;    

    // Last Keyframe inserted
    Keyframe* last_kf_inserted;
    // Mosaic frame
    Keyframe* mosaic_frame;

    // Mutex to control the access to the mosaic graph
    boost::mutex mutex_mgraph;

    // Adjuster for optimizing mosaic poses
    MosaicAdjuster madj;    

    // Variables to control the mosaicing process
    boost::mutex mutex_building;
    bool building;

    // Counters
    boost::mutex mutex_observ;
    int received_images;
    int obs_ok;
    int obs_nok;
};

}

#endif // MOSAICGRAPH_H
