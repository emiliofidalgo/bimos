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

double inline SQ(double x) { return x * x; }

/**
 * @brief Default constructor.
 */
MosaicGraph::MosaicGraph() :
    last_kf_inserted(0),
    mosaic_frame(0),
    building(false),
    received_images(0),
    obs_ok(0),
    obs_nok(0)
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
    if (last_kf_inserted != 0)
    {
        kf->trans = last_kf_inserted->trans * Transform(t);
    }
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

    // Notifying the existence of a new keyframe
    newKFs.push(kf);

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
 * @brief Adds constraints to the optimizer.
 * @param kf_prev Original KF.
 * @param kf Destination KF.
 * @param matches Inliers between the KFs.
 */
void MosaicGraph::addConstraints(Keyframe* kf_prev, Keyframe* kf, std::vector<cv::DMatch>& matches)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    madj.addConstraints(kf_prev, kf, matches);
}

/**
 * @brief Performs an optimization of the absolute positions of the graph.
 * @param summary Ceres solver summary.
 */
void MosaicGraph::optimize(ceres::Solver::Summary& summary, bool opt_local)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);

    ceres::Solver::Options solver_options;

    // Performing the optimization
    if (!opt_local)
    {
        // Global Optimization
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
        solver_options.max_num_iterations = 1000;
        solver_options.minimizer_progress_to_stdout = false;
        solver_options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );
        solver_options.num_linear_solver_threads = sysconf( _SC_NPROCESSORS_ONLN );
        //solver_options.parameter_tolerance = 0;
        //solver_options.function_tolerance = 0;
        //solver_options.gradient_tolerance = 0;
        solver_options.initial_trust_region_radius = 1e14;
        solver_options.max_solver_time_in_seconds = 600;
    }
    else
    {
        // Local Optimization
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
        solver_options.max_num_iterations = 50;
        solver_options.minimizer_progress_to_stdout = false;
        solver_options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );
        solver_options.num_linear_solver_threads = sysconf( _SC_NPROCESSORS_ONLN );
        solver_options.initial_trust_region_radius = 1e14;
        solver_options.max_solver_time_in_seconds = 30;
        //solver_options.parameter_tolerance = 0;
        //solver_options.function_tolerance = 0;
        //solver_options.gradient_tolerance = 0;
    }

    madj.adjust(solver_options, summary);

    // Updating the absolute homographies.
    for (unsigned kf_ind = 0; kf_ind < kfs.size(); kf_ind++)
    {
        kfs[kf_ind]->trans.updateHomography();
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

/**
 * @brief Returns a pointer to a keyframe of the graph.
 * @param id Keyframe ID.
 * @return A pointer to the KF.
 */
Keyframe* MosaicGraph::getKeyframe(const int id)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    return kfs[id];
}

/**
 * @brief Method for returning the number of KFs currently stored in the graph.
 * @return
 */
int MosaicGraph::getNumberOfKeyframes()
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    return static_cast<int>(kfs.size());
}

/**
 * @brief Returns a vector of the current transformation for each KF.
 * @param transforms An ordered vector of tranformations for each KF.
 */
void MosaicGraph::getKFTransforms(std::vector<Transform> &transforms)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    transforms.clear();
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        Transform t(kfs[i]->trans);
        transforms.push_back(t);
    }
}

/**
 * @brief Returns the description of the graph using the Dot language.
 * @param contents
 */
void MosaicGraph::getDotGraph(std::string& contents)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    std::stringstream ss;
    ss << "graph mgraph {" << std::endl;    

    if (kfs.size() == 0)
    {
        ss << "0 [label=\"0\", pos=\"0,0!\"];" << std::endl;
    }
    else
    {
        // Filling nodes
        for (unsigned i = 0; i < kfs.size(); i++)
        {            
            // Getting current node position
            int xpos = (int)(kfs[i]->trans.H(0, 2));
            int ypos = (int)(kfs[i]->trans.H(1, 2));

            std::stringstream t;
            t << kfs[i]->id << " [label=\"" << kfs[i]->id << "\", pos=\"" << xpos << "," << -ypos << "!\"];" << std::endl;
            ss << t.str();
        }

        // Filling edges
        cv::Mat_<char> eadded = cv::Mat::zeros((int)kfs.size(), (int)kfs.size(), CV_8UC1);
        for (unsigned i = 0; i < graph.adjlist.size(); i++)
        {
            for (unsigned j = 0; j < graph.adjlist[i].size(); j++)
            {
                int ori = i;
                int dest = graph.adjlist[i][j].target;
                if (eadded(dest, ori) == 0)
                {
                    std::stringstream t;
                    t << ori << " -- " << dest << ";" << std::endl;
                    ss << t.str();
                    eadded(ori, dest) = 1;
                }
            }
        }
    }

    ss << "}";
    ss << std::endl;
    //std::cout << ss.str() << std::endl; // TODO Comment this line. It is only for debugging.
    contents = ss.str();
}

/**
 * @brief Computes the error according to the current structure of the mosaic.
 * @param avg Average error in pixels.
 * @param stddev Standard deviation of the error in pixels.
 * @param inliers_dir Directory for loading inliers.
 */
void MosaicGraph::getMosaicError(double& avg, double& stddev, double& max, double& min, std::string& inliers_dir)
{
    boost::mutex::scoped_lock lock(mutex_mgraph);

    max = -1.0;
    min = std::numeric_limits<int>::max();

    // Total number of inliers
    long unsigned ninliers = 0;

    // Sums of values
    long double sum_x = 0.0;
    long double sum_x2 = 0.0;

    // Computing the error for the correspondences in each link
    for(std::map<int, std::map<int, Edge*> >::iterator outer_iter = edges.begin(); outer_iter != edges.end(); outer_iter++)
    {
        for(std::map<int, Edge*>::iterator inner_iter = outer_iter->second.begin(); inner_iter != outer_iter->second.end(); inner_iter++)
        {
            // Computing the error for this edge
            Edge* edg = inner_iter->second;
            int ori = edg->ori;
            int dest = edg->dest;

            // Computing the tranformation from ori to dest
            Transform t = kfs[ori]->trans.inv() * kfs[dest]->trans;

            // Loading the matchings of this link
            std::vector<cv::DMatch> inliers;
            loadMatchings(ori, dest, inliers_dir, inliers);

            // Incrementing the total number of inliers
            ninliers += static_cast<int>(inliers.size());

            // Iterating for each inlier
            for (unsigned inlier = 0; inlier < inliers.size(); inlier++)
            {
                double qidx = inliers[inlier].queryIdx;
                double tidx = inliers[inlier].trainIdx;

                double q_x = kfs[dest]->image->kps[qidx].pt.x;
                double q_y = kfs[dest]->image->kps[qidx].pt.y;
                double t_x = kfs[ori]->image->kps[tidx].pt.x;
                double t_y = kfs[ori]->image->kps[tidx].pt.y;

                double err = sqrt(SQ(t_x - (t.H(0,0) * q_x + t.H(0,1) * q_y + t.H(0,2))) + SQ(t_y - (t.H(1,0) * q_x + t.H(1,1) * q_y + t.H(1,2))));

                sum_x += err;
                sum_x2 += SQ(err);

                // Computing max and min
                if (err > max)
                {
                    max = err;
                }

                if (err < min)
                {
                    min = err;
                }
            }
        }
    }

    // Computing the average error and stddev
    avg = sum_x / static_cast<double>(ninliers);
    stddev = sqrt((sum_x2 / static_cast<double>(ninliers)) - SQ(avg));
}

/**
 * @brief Returns the current Mosaic time.
 * @return Returns the current mosaic time until this moment.
 */
double MosaicGraph::getMosaicTime()
{
    boost::mutex::scoped_lock lock(mutex_mgraph);
    return (last_kf_inserted->end_time - kfs[0]->init_time);
}

void MosaicGraph::setBuildingState(bool value)
{
    boost::mutex::scoped_lock lock(mutex_building);
    building = value;
    if (!building)
    {
        // Inserting a NULL value for stopping threads
        newKFs.push(0);
    }
}

bool MosaicGraph::isBuilding()
{
    boost::mutex::scoped_lock lock(mutex_building);
    return building;
}

/**
 * @brief Increments the total number of images received by the process.
 */
void MosaicGraph::incrNumImages()
{
    received_images++;
}

/**
 * @brief Returns the number of images received by the mosaicing process.
 * @return
 */
int MosaicGraph::getNumberOfImages()
{
    return received_images;
}

/**
 * @brief Increments the total number of successful observations.
 */
void MosaicGraph::incrObsOK()
{
    boost::mutex::scoped_lock lock(mutex_observ);
    obs_ok++;
}

/**
 * @brief Returns the total number of successful observations.
 * @return
 */
int MosaicGraph::getObsOK()
{
    boost::mutex::scoped_lock lock(mutex_observ);
    return obs_ok;
}

/**
 * @brief Increments the total number of unsuccessful observations.
 */
void MosaicGraph::incrObsNOK()
{
    boost::mutex::scoped_lock lock(mutex_observ);
    obs_nok++;
}

/**
 * @brief Returns the total number of unsuccessful observations.
 * @return
 */
int MosaicGraph::getObsNOK()
{
    boost::mutex::scoped_lock lock(mutex_observ);
    return obs_nok;
}

}
