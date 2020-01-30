#include "ViewerAR.h"

#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

namespace ORB_SLAM2 {
    ViewerAR::ViewerAR(const string settings) {
        cv::FileStorage fSettings(settings, cv::FileStorage::READ);
        float fps = fSettings["Camera.fps"];
        SetFPS(fps);

        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        SetCameraCalibration(fx,fy,cx,cy);
    }

    void ViewerAR::Run() {
        int w,h,wui;

        cv::Mat im, Tcw;
        int status;
        vector<cv::KeyPoint> vKeys;
        vector<MapPoint*> vMPs;

        while(1)
        {
            GetImagePose(im,Tcw,status,vKeys,vMPs);
            if(im.empty())
                cv::waitKey(mT);
            else
            {
                w = im.cols;
                h = im.rows;
                break;
            }
        }

        wui=200;

        pangolin::CreateWindowAndBind("Viewer",w+wui,h);

        glEnable(GL_DEPTH_TEST);
        glEnable (GL_BLEND);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(wui));
        pangolin::Var<bool> menu_detectplane("menu.Insert Cube",false,false);
        pangolin::Var<bool> menu_clear("menu.Clear All",false,false);
        pangolin::Var<bool> menu_drawim("menu.Draw Image",true,true);
        pangolin::Var<bool> menu_drawcube("menu.Draw Cube",true,true);
        pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
        pangolin::Var<bool> menu_drawgrid("menu.Draw Grid",true,true);
        pangolin::Var<int> menu_ngrid("menu. Grid Elements",3,1,10);
        pangolin::Var<float> menu_sizegrid("menu. Element Size",0.05,0.01,0.3);
        pangolin::Var<bool> menu_drawpoints("menu.Draw Points",false,true);

        pangolin::Var<bool> menu_LocalizationMode("menu.Localization Mode",false,true);
        bool bLocalizationMode = false;

        pangolin::View& d_image = pangolin::Display("image")
                .SetBounds(0,1.0f,pangolin::Attach::Pix(wui),1.0f,(float)w/h)
                .SetLock(pangolin::LockLeft, pangolin::LockTop);

        pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

        pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w,h,fx,fy,cx,cy,0.001,1000);

        vector<Plane*> vpPlane;

        while(1) {

            if (menu_LocalizationMode && !bLocalizationMode) {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            } else if (!menu_LocalizationMode && bLocalizationMode) {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Activate camera view
            d_image.Activate();
            glColor3f(1.0, 1.0, 1.0);


            // Get last image and its computed pose from SLAM
            GetImagePose(im, Tcw, status, vKeys, vMPs);

            // Add text to image
            PrintStatus(status, bLocalizationMode, im);

            if (menu_drawpoints)
                DrawTrackedPoints(vKeys, vMPs, im);

            // Draw image
            if (menu_drawim)
                DrawImageTexture(imageTexture, im);

            glClear(GL_DEPTH_BUFFER_BIT);

            // Load camera projection
            glMatrixMode(GL_PROJECTION);
            P.Load();

            glMatrixMode(GL_MODELVIEW);

            // Load camera pose
            LoadCameraPose(Tcw);

            // Draw virtual things
            if (status == 2) {
                if (menu_clear) {
                    if (!vpPlane.empty()) {
                        for (size_t i = 0; i < vpPlane.size(); i++) {
                            delete vpPlane[i];
                        }
                        vpPlane.clear();
                        cout << "All cubes erased!" << endl;
                    }
                    menu_clear = false;
                }
                if (menu_detectplane) {
                    Plane *pPlane = DetectPlane(Tcw, vMPs, 50);
                    if (pPlane) {
                        cout << "New virtual cube inserted!" << endl;
                        vpPlane.push_back(pPlane);
                    } else {
                        cout << "No plane detected. Point the camera to a planar region." << endl;
                    }
                    menu_detectplane = false;
                }

                if (!vpPlane.empty()) {
                    // Recompute plane if there has been a loop closure or global BA
                    // In localization mode, map is not updated so we do not need to recompute
                    bool bRecompute = false;
                    if (!bLocalizationMode) {
                        if (mpSystem->MapChanged()) {
                            cout << "Map changed. All virtual elements are recomputed!" << endl;
                            bRecompute = true;
                        }
                    }

                    for (size_t i = 0; i < vpPlane.size(); i++) {
                        Plane *pPlane = vpPlane[i];

                        if (pPlane) {
                            if (bRecompute) {
                                pPlane->Recompute();
                            }
                            glPushMatrix();
                            pPlane->glTpw.Multiply();

                            // Draw cube
                            if (menu_drawcube) {
                                DrawCube(menu_cubesize);
                            }

                            // Draw grid plane
                            if (menu_drawgrid) {
                                DrawPlane(menu_ngrid, menu_sizegrid);
                            }

                            glPopMatrix();
                        }
                    }
                }


            }

            pangolin::FinishFrame();
            usleep(mT * 1000);
        }
    }

    void ViewerAR::SetCurrentCameraPoseMapDrawer(const cv::Mat &Tcw) {
        unique_lock<mutex> lock(mMutexPoseImage);
        mTcw = Tcw;
    }

    void ViewerAR::UpdateFrameDrawer(ORB_SLAM2::Tracking *pTracker) {
        unique_lock<mutex> lock(mMutexPoseImage);
        mStatus = mpSystem->GetTrackingState();
        mvKeys = pTracker->mCurrentFrame.mvKeys;
        mvMPs =  pTracker->mCurrentFrame.mvpMapPoints;
        mImage = pTracker->mImGray;
    }

    void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
    {
        unique_lock<mutex> lock(mMutexPoseImage);
        im = mImage.clone();
        Tcw = mTcw.clone();
        status = mStatus;
        vKeys = mvKeys;
        vMPs = mvMPs;
    }

    void ViewerAR::RequestFinish() {
        return;
    }

    bool ViewerAR::isFinished() {
        return false;
    }

    bool ViewerAR::isStopped() {
        return false;
    }

    void ViewerAR::Release() {
        return;
    }

    void ViewerAR::SetMap(ORB_SLAM2::Map *map) {
        return;
    }

    void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
    {
        if(!Tcw.empty())
        {
            pangolin::OpenGlMatrix M;

            M.m[0] = Tcw.at<float>(0,0);
            M.m[1] = Tcw.at<float>(1,0);
            M.m[2] = Tcw.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Tcw.at<float>(0,1);
            M.m[5] = Tcw.at<float>(1,1);
            M.m[6] = Tcw.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Tcw.at<float>(0,2);
            M.m[9] = Tcw.at<float>(1,2);
            M.m[10] = Tcw.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = Tcw.at<float>(0,3);
            M.m[13] = Tcw.at<float>(1,3);
            M.m[14] = Tcw.at<float>(2,3);
            M.m[15]  = 1.0;

            M.Load();
        }
    }

    void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
    {
        if(!bLocMode)
        {
            switch(status)
            {
                case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
                case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
                case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
            }
        }
        else
        {
            switch(status)
            {
                case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
                case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
                case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
            }
        }
    }

    void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
    {
        int l = 10;
        //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
        cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
        cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

        cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
    }

    void ViewerAR::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
    {
        if(!im.empty())
        {
            imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
            imageTexture.RenderToViewportFlipY();
        }
    }

    void ViewerAR::DrawCube(const float &size,const float x, const float y, const float z)
    {
        pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
        glPushMatrix();
        M.Multiply();
        pangolin::glDrawColouredCube(-size,size);
        glPopMatrix();
    }

    void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
    {
        glPushMatrix();
        pPlane->glTpw.Multiply();
        DrawPlane(ndivs,ndivsize);
        glPopMatrix();
    }

    void ViewerAR::DrawPlane(int ndivs, float ndivsize)
    {
        // Plane parallel to x-z at origin with normal -y
        const float minx = -ndivs*ndivsize;
        const float minz = -ndivs*ndivsize;
        const float maxx = ndivs*ndivsize;
        const float maxz = ndivs*ndivsize;


        glLineWidth(2);
        glColor3f(0.7f,0.7f,1.0f);
        glBegin(GL_LINES);

        for(int n = 0; n<=2*ndivs; n++)
        {
            glVertex3f(minx+ndivsize*n,0,minz);
            glVertex3f(minx+ndivsize*n,0,maxz);
            glVertex3f(minx,0,minz+ndivsize*n);
            glVertex3f(maxx,0,minz+ndivsize*n);
        }

        glEnd();
    }

    void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs, cv::Mat &im)
    {
        const int N = vKeys.size();


        for(int i=0; i<N; i++)
        {
            if(vMPs[i])
            {
                cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
            }
        }
    }

    Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
    {
        return mpSystem->DetectPlane(iterations);
    }

    void ViewerAR::SetTracking(const ORB_SLAM2::Tracking *tracking) {
        return;
    }

    void ViewerAR::RequestStop() {
        return;
    }

}

