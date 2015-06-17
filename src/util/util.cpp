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

#include <bimos/util/util.h>

namespace bimos
{

/**
 * @brief Performs a brute force matching from \a query to \a train descriptors, and then filters the resuls using NNDR.
 * @param query Input query image.
 * @param train Input train image.
 * @param matches Final filtered matches.
 * @param ratio Distance ratio between the nearest and the second-nearest neigbours.
 */
void ratioMatching(Image* query, Image* train, std::vector<cv::DMatch>& matches, const double ratio)
{
    matches.clear();
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // Matching descriptors.
    std::vector<std::vector<cv::DMatch> > matches12;
    matcher.knnMatch(query->dscs, train->dscs, matches12, 2);

    // Filtering the resulting matchings according to the given ratio.
    for (unsigned m = 0; m < matches12.size(); m++)
    {
        if (matches12[m][0].distance < matches12[m][1].distance * ratio)
        {
            matches.push_back(matches12[m][0]);
        }
    }
}

/**
 * @brief Gets an ordered list of the images stored in \a directory.
 * @param directory Directory to search for images.
 * @param filenames Resulting vector of image filenames.
 */
void getImageFilenames(const std::string& directory, std::vector<std::string>& filenames)
{
    using namespace boost::filesystem;

    filenames.clear();
    path dir(directory);

    // Retrieving, sorting and filtering filenames.
    std::vector<path> entries;
    copy(directory_iterator(dir), directory_iterator(), back_inserter(entries));
    sort(entries.begin(), entries.end());
    for (std::vector<path>::const_iterator it(entries.begin()); it != entries.end(); ++it)
    {
        std::string ext = it->extension().c_str();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".jpg" || ext == ".ppm" || ext == ".jpeg")
        {
            filenames.push_back(it->string());
        }
    }
}

/**
 * @brief Stores matchings on disk.
 * @param ori Origin KF.
 * @param dest Destination  KF.
 * @param dir Directory to store the file.
 * @param matches The matching vector.
 */
void saveMatchings(const int ori, const int dest, const std::string& dir, const std::vector<cv::DMatch>& matches)
{
    // Saving matchings in forward direction
    char name[500];
    std::string yfilename = dir + "matchings%06d_%06d.yml";
    sprintf(name, yfilename.c_str(), ori, dest);

    cv::FileStorage fs1(std::string(name), cv::FileStorage::WRITE);
    fs1 << "matches" << "[";
    for (unsigned m = 0; m < matches.size(); m++)
    {
        fs1 << "{:";
        fs1 << "q" << matches[m].queryIdx;
        fs1 << "t" << matches[m].trainIdx;
        fs1 << "i" << matches[m].imgIdx;
        fs1 << "d" << matches[m].distance;
        fs1 << "}";
    }
    fs1 << "]";
    fs1.release();

    // Saving matchings in backward direction
    // FIXME
    sprintf(name, yfilename.c_str(), dest, ori);

    cv::FileStorage fs2(std::string(name), cv::FileStorage::WRITE);
    fs2 << "matches" << "[";
    for (unsigned m = 0; m < matches.size(); m++)
    {
        fs2 << "{:";
        fs2 << "q" << matches[m].trainIdx;
        fs2 << "t" << matches[m].queryIdx;
        fs2 << "i" << matches[m].imgIdx;
        fs2 << "d" << matches[m].distance;
        fs2 << "}";
    }
    fs2 << "]";
    fs2.release();
}

/**
 * @brief Loads matchings from disk.
 * @param ori Origin KF.
 * @param dest Destination KF.
 * @param dir Directory from where to load the file.
 * @param matches The matching vector.
 */
void loadMatchings(const int ori, const int dest, const std::string& dir, std::vector<cv::DMatch>& matches)
{
    char name[500];
    std::string yfilename = dir + "matchings%06d_%06d.yml";
    sprintf(name, yfilename.c_str(), ori, dest);

    cv::FileStorage fs(std::string(name), cv::FileStorage::READ);

    cv::FileNode mtchs = fs["matches"];
    cv::FileNodeIterator it = mtchs.begin(), it_end = mtchs.end();
    matches.clear();
    for(; it != it_end; it++)
    {
        cv::DMatch match;
        match.queryIdx = (int)(*it)["q"];
        match.trainIdx = (int)(*it)["t"];
        match.imgIdx = (int)(*it)["i"];
        match.distance = (float)(*it)["d"];

        matches.push_back(match);
    }

    fs.release();
}

}
