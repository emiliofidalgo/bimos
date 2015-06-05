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
void ratioMatching(Image* query, Image* train, std::vector<cv::DMatch>& matches, const float ratio)
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

}
