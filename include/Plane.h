#ifndef ORB_SLAM2_PLANE_H
#define ORB_SLAM2_PLANE_H

#include <opencv2/core/core.hpp>
#ifndef EMSCRIPTEN
#include "pangolin/pangolin.h"
#endif
#include "MapPoint.h"

namespace ORB_SLAM2
{
    class Plane
    {
    public:
        Plane(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);
        Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

        void Recompute();

        //normal
        cv::Mat n;
        //origin
        cv::Mat o;
        //arbitrary orientation along normal
        float rang;
        //transformation from world to the plane
        cv::Mat Tpw;

        const cv::Mat &GetTpw() const;

#ifndef EMSCRIPTEN
        pangolin::OpenGlMatrix glTpw;
#endif
        //MapPoints that define the plane
        std::vector<MapPoint*> mvMPs;
        //camera pose when the plane was first observed (to compute normal direction)
        cv::Mat mTcw, XC;

        static cv::Mat ExpSO3(const float &x, const float &y, const float &z);

        static cv::Mat ExpSO3(const cv::Mat &v);

        constexpr static const float eps = 1e-4;

    };
}

#endif //ORB_SLAM2_PLANE_H
