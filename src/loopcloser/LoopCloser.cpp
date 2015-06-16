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
    mgraph(_mgraph),
    bindex_init(false)
{    
    vi::BinaryIndexParams bparams;
    bparams.max_descriptors = p->nkeypoints * 10000; // Assuming a maximum of 10000 images. Be careful with this value!
    bindex = new vi::BinaryIndex(bparams);
}

/**
 * @brief Default destructor.
 */
LoopCloser::~LoopCloser()
{    
    delete bindex;
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

        // Inserting the current KF in the delay buffer
        buffer.push(newkf);

        // Inserting new hypothesis as loop candidates from the delay buffer
        if (buffer.size() == p->lc_delay_kfs)
        {
            Keyframe* loopcand = buffer.front();
            buffer.pop();

            // Adding the image as possible loop closure candidate to the index
            ROS_INFO("[loopcloser] Inserting KF %i as LC candidate", loopcand->id);
            if (!bindex_init)
            {
                bindex->add(0, loopcand->image->kps, loopcand->image->dscs);
                bindex_init = true;
            }
            else
            {
                // Matching the current image against the index
                std::vector<std::vector<cv::DMatch> > matches_feats;
                bindex->search(loopcand->image->dscs, matches_feats, 2);

                // Filtering matches
                std::vector<cv::DMatch> matches;
                for (unsigned m = 0; m < matches_feats.size(); m++)
                {
                    if (matches_feats[m][0].distance < matches_feats[m][1].distance * p->match_ratio)
                    {
                        matches.push_back(matches_feats[m][0]);
                    }
                }

                // Updating the index
                bindex->update(loopcand->id, loopcand->image->kps, loopcand->image->dscs, matches);
            }
        }

        // Searching for loop closures
        if (bindex_init)
        {
            ROS_INFO("[loopcloser] Searching overlapping frames for KF %i", newkf->id);

            // Matching the current descriptors againts the index
            std::vector<std::vector<cv::DMatch> > matches_feats;
            bindex->search(newkf->image->dscs, matches_feats, 2);

            // Filtering the matchings
            std::vector<cv::DMatch> matches;
            for (unsigned m = 0; m < matches_feats.size(); m++)
            {
                if (matches_feats[m][0].distance < matches_feats[m][1].distance * p->match_ratio)
                {
                    matches.push_back(matches_feats[m][0]);
                }
            }

            // Getting similar images according to the feature matchings
            std::vector<vi::ImageMatch> image_matches;
            bindex->getSimilarImages(newkf->image->dscs, matches, image_matches);

            // Processing the loop closure candidates
            for (unsigned i = 0; i < image_matches.size(); i++)
            {
                int cand_id = image_matches[i].image_id;
                double score = image_matches[i].score;

                // Computing the transformation between the current KF and the candidate KF
                Keyframe* cand_kf = mgraph->getKeyframe(cand_id);
                cv::Mat_<double> H;
                std::vector<cv::DMatch> inliers;
                double rep_error;
                HomographyEstimator::estimate(newkf->image, cand_kf->image, H, inliers, rep_error, p->match_ratio);
                ROS_INFO("[loopdetector] Candidate %i, Score %f, Inliers %i, MRE: %f", cand_id, score, static_cast<int>(inliers.size()), rep_error);

                // Detecting if the number of inliers is higher than the indicated threshold
                if (inliers.size() > p->min_inliers)
                {
                    // Linking the images in the graph
                    mgraph->linkKFs(newkf->id, cand_id, rep_error, H);
                    ROS_INFO("[loopdetector] --- Loop detected!, Linking KFs %i and %i", newkf->id, cand_id);
                }
                else
                {
                    //Otherwise, we stop to search loop candidates
                    break;
                }
            }
        }

        // Sleeping the needed time
        ros::spinOnce();
        r.sleep();
    }
}

}
