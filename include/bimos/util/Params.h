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

#include <bimos/util/util.h>

namespace bimos
{

/**
 * @brief Singleton class to store the parameters of the application.
 */
class Params
{
public:
    // Parameters    
    int nimages;    
    std::string working_dir;
    std::string img_descriptor;
    int nkeypoints;

    // Public functions.
    static Params* getInstance();
    void readParams(const ros::NodeHandle& nh);

protected:
    // Protected constructor. Singleton class.
    Params() :
        nimages(0),
        nkeypoints(2500)
    {
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
