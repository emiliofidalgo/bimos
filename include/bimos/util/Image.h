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

#ifndef IMAGEBIMOS_H
#define IMAGEBIMOS_H

#include <opencv2/opencv.hpp>

namespace bimos
{

/**
 * @brief Struct used to store all information related with an image.
 */
struct Image
{
    int id;
    std::string filename;
    cv::Mat image;
    std::vector<cv::KeyPoint> kps;
    cv::Mat dscs;

    void save(const std::string& file)
    {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);

        fs << "id" << id;
        fs << "filename" << filename;
        cv::write(fs, "kps", kps);
        fs << "dscs" << dscs;

        fs.release();
    }

    void load(const std::string& file)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);

        id = fs["id"];
        filename = (std::string)fs["filename"];
        image = cv::imread(filename);
        kps.clear();
        cv::read(fs["kps"], kps);
        fs["dscs"] >> dscs;

        fs.release();
    }
};

}

#endif // IMAGEBIMOS_H
