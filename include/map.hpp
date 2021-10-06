#ifndef MAP_HPP
#define MAP_HPP

#include <vector>
#include <set>

#include <QMutex>

namespace MY_SLAM
{
class KeyFrame;
class MapPoint;

class Map
{
public:
  Map();

  void addKeyFrame(KeyFrame* pKF);
  void addMapPoint(MapPoint* pMP);
  void eraseMapPoint(MapPoint* pMP);
  void eraseKeyFrame(KeyFrame* pKF);
  void setReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
  void informNewBigChange();
  int getLastBigChangeIdx();

  std::vector<KeyFrame*> getAllKeyFrames();
  std::vector<MapPoint*> getAllMapPoints();
  std::vector<MapPoint*> getReferenceMapPoints();

  long unsigned int mapPointsInMap();
  long unsigned  keyFramesInMap();

  long unsigned int getMaxKFid();

  void clear();

  std::vector<KeyFrame*> mvpKeyFrameOrigins;

  QMutex mMutexMapUpdate;

  // This avoid that two points are created simultaneously in separate threads (id conflict)
  QMutex mMutexPointCreation;

protected:
  std::set<MapPoint*> mspMapPoints;
  std::set<KeyFrame*> mspKeyFrames;

  std::vector<MapPoint*> mvpReferenceMapPoints;

  long unsigned int mnMaxKFid;

  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;

  QMutex mMutexMap;
};
} // namespace MY_SLAM

#endif // MAP_HPP
