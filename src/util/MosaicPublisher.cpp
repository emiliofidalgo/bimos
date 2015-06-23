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

#include <bimos/util/MosaicPublisher.h>

namespace bimos
{

MosaicPublisher::MosaicPublisher(const ros::NodeHandle _nh, bimos::MosaicGraph *graph, bimos::Params *_p) :
    nh(_nh),
    it(nh),
    mgraph(graph),
    p(_p)
{
    pub_graph = it.advertise("mosaic_graph", 1);
}

void MosaicPublisher::publishGraphInfo()
{
    // Getting graph information
    std::string dot_graph;
    mgraph->getDotGraph(dot_graph);

    // Storing the .dot file
    std::string mgraph_file = p->working_dir + "mgraph.dot";
    std::ofstream out(mgraph_file.c_str());
    out << dot_graph;
    out.close();

    // Calling to the conversion function
    std::string command = "dot -Kneato -n -Tjpg " + p->working_dir + "mgraph.dot -o " + p->working_dir + "mgraph.jpg";
    std::system(command.c_str());

    // Publishing the image
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "mosaic_graph";
    cv::Mat img_graph = cv::imread(p->working_dir + "mgraph.jpg");

    sensor_msgs::Image img_msg;
    cv_bridge::CvImage cv_image(hdr, "bgr8", img_graph);
    cv_image.toImageMsg(img_msg);
    pub_graph.publish(img_msg);
}

}
