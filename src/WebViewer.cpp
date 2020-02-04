#include "WebViewer.h"

namespace ORB_SLAM2
{
    WebViewer::WebViewer():isStop(false) {
        frameDrawer = new FrameDrawer();
    }

    void WebViewer::Run() {}

    void WebViewer::RequestStop()
    {
        isStop = true;
    }

    bool WebViewer::isStopped()
    {
        return isStop;
    }

    void WebViewer::RequestFinish() {}

    bool WebViewer::isFinished() {
        return true;
    }

    void WebViewer::SetTracking(const ORB_SLAM2::Tracking *tracking) {}

    void WebViewer::SetMap(ORB_SLAM2::Map *map) {
        frameDrawer->SetMap(map);
    }

    void WebViewer::SetCurrentCameraPoseMapDrawer(const cv::Mat &Tcw) {}

    void WebViewer::UpdateFrameDrawer(ORB_SLAM2::Tracking *pTracker) {
        frameDrawer->Update(pTracker);
    }

    void WebViewer::Release() {
        isStop = false;
    }

    cv::Mat WebViewer::DrawFrame() {
        return frameDrawer->DrawFrame();
    }


}

