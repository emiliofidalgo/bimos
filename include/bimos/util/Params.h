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

#ifndef _PARAMS_H
#define _PARAMS_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <bimos/util/util.h>
#include <bimos/BimosConfig.h>

namespace bimos
{

/**
 * @brief Singleton class to store the parameters of the application.
 */
class Params
{
public:
    // Parameters
    std::string working_dir;
    std::string img_descriptor;
    int nkeypoints;
    bool pub_debug_info;
    int lc_delay_kfs;
    double match_ratio;
    int min_inliers;
    int optim_every_kfs;
    bool blend_exp;
    bool blend_seams;
    int kf_min_inliers;
    double kf_overlap;
    bool batch;
    std::string batch_images_dir;
    double max_reproj_error;

    dynamic_reconfigure::Server<bimos::BimosConfig> server;
    dynamic_reconfigure::Server<bimos::BimosConfig>::CallbackType f;

    // Public functions.
    static Params* getInstance();
    void readParams(const ros::NodeHandle& nh);
    void modifyParams(bimos::BimosConfig& config, uint32_t level);

protected:
    // Protected constructor. Singleton class.
    Params() :        
        nkeypoints(2500),
        pub_debug_info(false),
        lc_delay_kfs(5),
        match_ratio(0.8),
        min_inliers(200),
        optim_every_kfs(15),
        blend_exp(false),
        kf_min_inliers(550),
        kf_overlap(0.4),
        batch(false),
        blend_seams(false),
        max_reproj_error(3.0)
    {
        f = boost::bind(&Params::modifyParams, this, _1, _2);
        server.setCallback(f);
    }

    ~Params()
    {
        delete _instance;
    }

    Params(const Params &);
    Params& operator=(const Params &);

private:
    // Single instance.
    static Params* _instance;
};

}
#endif /* _PARAMS_H */
