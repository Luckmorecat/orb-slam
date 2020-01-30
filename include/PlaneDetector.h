//
// Created by alen on 28.01.20.
//

#ifndef ORB_SLAM2_PLANEDETECTOR_H
#define ORB_SLAM2_PLANEDETECTOR_H

#include "Plane.h"
#include "opencv2/core/core.hpp"

namespace ORB_SLAM2
{

    class PlaneDetector
    {
    public:
        Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations);
    };

}

#endif //ORB_SLAM2_PLANEDETECTOR_H
