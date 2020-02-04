

#ifndef ORB_SLAM2_VIEWERAR_H
#define ORB_SLAM2_VIEWERAR_H

#include "IViewer.h"
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include "mutex"
#include "Plane.h"
#include "string"

namespace ORB_SLAM2
{

    class ViewerAR: public IViewer
    {
    public:
        ViewerAR(string settings);

        // Main thread function.
        void Run() override;

//        void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
//                          const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs);

        void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                          std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs);

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

        //SLAM
        ORB_SLAM2::System* mpSystem;

        void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);
        void AddTextToImage(const std::string &s, cv::Mat &im, const int r=0, const int g=0, const int b=0);
        void LoadCameraPose(const cv::Mat &Tcw);
        void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);
        void DrawCube(const float &size, const float x=0, const float y=0, const float z=0);
        void DrawPlane(int ndivs, float ndivsize);
        void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);
        void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs, cv::Mat &im);

        Plane* DetectPlane(const int iterations=50);

        // frame rate
//        float mFPS, mT;
//        float fx,fy,cx,cy;

        // Last processed image and computed pose by the SLAM
        std::mutex mMutexPoseImage;
        cv::Mat mTcw;
        cv::Mat mImage;
        int mStatus;
        std::vector<cv::KeyPoint> mvKeys;
        std::vector<MapPoint*> mvMPs;

        std::mutex mMutexStop;
        bool isStop;
    };
}

#endif //ORB_SLAM2_VIEWERAR_H
