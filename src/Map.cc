/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#if defined WITHTHREAD || defined BUILDNATIVE
#include<mutex>
#endif

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    #if defined WITHTHREAD || defined BUILDNATIVE
unique_lock<mutex> lock(mMutexMap);
#endif
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
