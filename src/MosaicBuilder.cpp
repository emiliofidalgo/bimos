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

#include "bimos/MosaicBuilder.h"

namespace bimos
{

/**
 * @brief Default class constructor.
 * @param nh ROS node handle.
 */
MosaicBuilder::MosaicBuilder(const ros::NodeHandle _nh)
    : nh(_nh),
      p(Params::getInstance())
{
    ROS_INFO("Initializing node ...");
    ROS_INFO("Reading parameters ...");
    p->readParams(nh);
    ROS_INFO("Parameters read");
    ROS_INFO("Node initialized");

    createMosaic();
}

/**
 * @brief Default class destructor.
 */
MosaicBuilder::~MosaicBuilder()
{    
}

/**
 * @brief Starts the mosaicing process using the options stored in params.
 */
void MosaicBuilder::createMosaic()
{
    KeyframeSelector kfsel(nh);
    boost::thread kfsel_thread(&KeyframeSelector::run, &kfsel);

    kfsel_thread.join();

   /*ImageDescriptor imgdes(p->img_descriptor, p->nkeypoints);

   for (int i = 0; i < p->nimages; i++)
   {
       cv::Mat img = cv::imread(p->img_filenames[i], 0);
       std::vector<cv::KeyPoint> kps;
       cv::Mat dscs;
       imgdes.describeImage(img, kps, dscs);

       std::cout << "Kps: " << kps.size() << std::endl;

       cv::Mat outimg;
       cv::drawKeypoints(img, kps, outimg);

       cv::imshow("Kps", outimg);
       cv::waitKey(0);
   }*/
}

}
