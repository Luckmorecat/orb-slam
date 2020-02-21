#include "PlaneDetector.h"

namespace ORB_SLAM2
{
    Plane* PlaneDetector::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint *> &vMPs, const int iterations) {
        vector<cv::Mat> vPoints;
        vPoints.reserve(vMPs.size());
        vector<MapPoint*> vPointMP;
        vPointMP.reserve(vMPs.size());

        for(size_t i=0; i<vMPs.size(); i++)
        {
            MapPoint* pMP=vMPs[i];
            if(pMP)
            {
                if(pMP->Observations()>5)
                {
                    vPoints.push_back(pMP->GetWorldPos());
                    vPointMP.push_back(pMP);
                }
            }
        }

        const int N = vPoints.size();

        if(N<50)
            return nullptr;


        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        float bestDist = 1e10;
        vector<float> bestvDist;

        //RANSAC
        for(int n=0; n<iterations; n++)
        {
            vAvailableIndices = vAllIndices;

            cv::Mat A(3,4,CV_32F);
            A.col(3) = cv::Mat::ones(3,1,CV_32F);

            // Get min set of points
            for(short i = 0; i < 3; ++i)
            {
                int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

                int idx = vAvailableIndices[randi];

                A.row(i).colRange(0,3) = vPoints[idx].t();

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }

            cv::Mat u,w,vt;
            cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

            const float a = vt.at<float>(3,0);
            const float b = vt.at<float>(3,1);
            const float c = vt.at<float>(3,2);
            const float d = vt.at<float>(3,3);

            vector<float> vDistances(N,0);

            const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

            for(int i=0; i<N; i++)
            {
                vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
            }

            vector<float> vSorted = vDistances;
            sort(vSorted.begin(),vSorted.end());

            int nth = max((int)(0.2*N),20);
            const float medianDist = vSorted[nth];

            if(medianDist<bestDist)
            {
                bestDist = medianDist;
                bestvDist = vDistances;
            }
        }

        // Compute threshold inlier/outlier
        const float th = 1.4*bestDist;
        vector<bool> vbInliers(N,false);
        int nInliers = 0;
        for(int i=0; i<N; i++)
        {
            if(bestvDist[i]<th)
            {
                nInliers++;
                vbInliers[i]=true;
            }
        }

        vector<MapPoint*> vInlierMPs(nInliers,NULL);
        int nin = 0;
        for(int i=0; i<N; i++)
        {
            if(vbInliers[i])
            {
                vInlierMPs[nin] = vPointMP[i];
                nin++;
            }
        }

        return new Plane(vInlierMPs,Tcw);
    }
}

