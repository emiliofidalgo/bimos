#ifndef _POINT2D_H_
#define _POINT2D_H_

#include <iostream>

namespace bimos
{

/**
 * Primitive 2D point class used as input for the LineParamEstimator.
 *
 * Author: Ziv Yaniv (zivy@cs.huji.ac.il)
 */
class Point2D {
public:
    Point2D(double px = 0.0, double py = 0.0) : x(px), y(py) {}
    double x;
    double y;
};

typedef std::pair<Point2D,Point2D> PointPair2D;

inline std::ostream &operator<<(std::ostream &output,const Point2D &pnt)
{
    output<<pnt.x<<' '<<pnt.y;
    return(output);
}

}

#endif //_POINT2D_H_
