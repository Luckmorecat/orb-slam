//
// Created by alen on 28.01.20.
//

#ifndef ORB_SLAM2_IVIEWER_H
#define ORB_SLAM2_IVIEWER_H

#include "System.h"
#include "Tracking.h"
#include "opencv2/core/core.hpp"
#include "Map.h"


namespace ORB_SLAM2
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class IViewer {
    public:
        void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_){
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        void SetFPS(const float fps){
            mFPS = fps;
            mT = 1e3/fps;
        }

        void SetSLAM(ORB_SLAM2::System* pSystem) {
            mpSystem = pSystem;
        }

        virtual void Run() = 0;

        virtual void RequestFinish() = 0;

        virtual void RequestStop() = 0;

        virtual bool isFinished() = 0;

        virtual bool isStopped() = 0;

        virtual void Release() = 0;

        virtual void UpdateFrameDrawer(Tracking *pTracker) = 0;

        virtual void SetCurrentCameraPoseMapDrawer(const cv::Mat &Tcw) = 0;

        virtual void SetMap(Map *map) = 0;

        virtual void SetTracking(const Tracking *tracking) = 0;

        virtual cv::Mat DrawFrame() = 0;

    protected:
        // frame rate
        float mFPS, mT;
        float fx,fy,cx,cy;

        System* mpSystem = nullptr;

    };

}




#endif //ORB_SLAM2_IVIEWER_H
