#include "map.hpp"
#include "keyframe.hpp"

namespace MY_SLAM
{
Map::Map()
  : mnMaxKFid(0), mnBigChangeIdx(0)
{

}

void Map::addKeyFrame(KeyFrame *pKF)
{
    QMutexLocker locker(&mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::addMapPoint(MapPoint *pMP)
{
    QMutexLocker locker(&mMutexMap);
    mspMapPoints.insert(pMP);
}

std::vector<KeyFrame*> Map::getAllKeyFrames()
{
  QMutexLocker locker(&mMutexMap);
  return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}
} // namespace MY_SLAM
