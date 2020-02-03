#ifndef ORB_SLAM2_WEBVIEWER_H
#define ORB_SLAM2_WEBVIEWER_H

#include "IViewer.h"
#include "FrameDrawer.h"
#include "opencv2/core/core.hpp"

namespace ORB_SLAM2
{
    class WebViewer: public IViewer
    {
    public:
        WebViewer();

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

    private:
        bool isStop;

        FrameDrawer *frameDrawer;
    };
}


#endif //ORB_SLAM2_WEBVIEWER_H


