#ifndef ORB_SLAM2_WEBVIEWER_H
#define ORB_SLAM2_WEBVIEWER_H

#include "opencv2/core/core.hpp"
#include "FrameDrawer.h"
#include "IViewer.h"


namespace ORB_SLAM2
{

    class Tracking;
    class FrameDrawer;
    class System;
    class IViewer;
    class Map;

    class WebViewer: public IViewer
    {
    public:
        WebViewer();

        WebViewer(const WebViewer &webViewer);

        void Run() override;

        void RequestFinish() override;

        void RequestStop() override;

        bool isFinished() override;

        bool isStopped() override;

        void Release() override;

        void UpdateFrameDrawer(Tracking *pTracker) override;

        void SetCurrentCameraPoseMapDrawer(const cv::Mat &Tcw) override;

        void SetMap(Map *map) override;

        void SetTracking(const Tracking *tracking) override;

        cv::Mat DrawFrame() override;

        FrameDrawer* GetFrameDrawer();

        System* GetSystem();

    private:
        bool isStop;

        FrameDrawer *frameDrawer;
    };
}


#endif //ORB_SLAM2_WEBVIEWER_H


