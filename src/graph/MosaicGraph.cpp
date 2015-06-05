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

#include "bimos/graph/MosaicGraph.h"

namespace bimos
{

/**
 * @brief Default constructor.
 */
MosaicGraph::MosaicGraph() :
    last_kf_inserted(0),
    mosaic_frame(0)
{
}

/**
 * @brief Default destructor for freeing memory.
 */
MosaicGraph::~MosaicGraph()
{
    // Deleting keyframes
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        delete kfs[i];
    }

    // Deleting edges
    for(std::map<int, std::map<int, Edge*> >::iterator outer_iter = edges.begin(); outer_iter != edges.end(); outer_iter++)
    {
        for(std::map<int, Edge*>::iterator inner_iter = outer_iter->second.begin(); inner_iter != outer_iter->second.end(); inner_iter++)
        {
            delete inner_iter->second;
        }
    }
}

/**
 * @brief Adds a new keyframe to the mosaic graph and links with the last inserted KF if needed.
 * @param img A pointer to the image.
 * @param weight Link weight.
 * @param t The tranformation from the last inserted keyframe to the current keyframe.
 */
int MosaicGraph::addKeyframe(Image* img, const double weight, const cv::Mat& t)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);

    // Creating and adding the new keyframe structure
    Keyframe* kf = new Keyframe(img);
    kf->id = static_cast<int>(kfs.size());
    kfs.push_back(kf);
    graph.addVertex();

    // Linking the new keyframe with the last inserted, if needed
    if (last_kf_inserted != 0)
    {
        int a = last_kf_inserted->id;
        int b = kf->id;

        // Linking images in both directions
        if (!existsEdge(a, b))
        {
            Edge* medge = new Edge(a, b, weight, t);
            edges[a][b] = medge;
            graph.addEdge(a, b, weight);
        }

        if (!existsEdge(b, a))
        {
            Edge* medge = new Edge(b, a, weight, t.inv());
            edges[b][a] = medge;
            graph.addEdge(b, a, weight);
        }
    }
    else
    {
        mosaic_frame = kf;
    }

    // Updating the last KF inserted
    last_kf_inserted = kf;

    return kf->id;
}

/**
 * @brief Creates a link between two keyframes.
 * @param a Origin KF.
 * @param b Destination KF.
 * @param weight Link weight.
 * @param t The tranformation from \a to \b.
 */
void MosaicGraph::linkKFs(const int a, const int b, const double weight, const cv::Mat& t)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);

    // Linking images in both directions
    if (!existsEdge(a, b))
    {
        Edge* medge = new Edge(a, b, weight, t);
        edges[a][b] = medge;
        graph.addEdge(a, b, weight);
    }

    if (!existsEdge(b, a))
    {
        Edge* medge = new Edge(b, a, weight, t.inv());
        edges[b][a] = medge;
        graph.addEdge(b, a, weight);
    }
}

/**
 * @brief Determines if a link exists in the graph.
 * @param ori Origin KF.
 * @param dest Destination KF.
 * @return \a true if the link exitst; \a false otherwise.
 */
bool MosaicGraph::existsEdge(const int ori, const int dest)
{
    std::map<int, std::map<int, Edge*> >::const_iterator i = edges.find(ori);
    if (i == edges.end()) return false;
    std::map<int, Edge*>::const_iterator j = i->second.find(dest);
    return j != (i->second.end());
}

/**
 * @brief Gets the last inserted keyframe.
 * @return Returns a pointer to the last inserted keyframe.
 */
Keyframe* MosaicGraph::getLastInsertedKF()
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    return last_kf_inserted;
}

/**
 * @brief Gets the mosaic frame.
 * @return Returns a pointer to the mosaic frame.
 */
Keyframe* MosaicGraph::getMosaicFrame()
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    return mosaic_frame;
}

}
