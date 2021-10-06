#ifndef MAPPOINT_HPP
#define MAPPOINT_HPP

#include <map>

#include <opencv2/core/core.hpp>

#include <QMutex>

namespace MY_SLAM
{
class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
//  MapPoint();
  MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
  MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

  void setWorldPos(const cv::Mat &Pos);
  cv::Mat getWorldPos();

//  cv::Mat getNormal();
//  KeyFrame* getReferenceKeyFrame();

//  std::map<KeyFrame*,size_t> getObservations();
  int observations();

  void addObservation(KeyFrame* pKF,size_t idx);
//  void eraseObservation(KeyFrame* pKF);

//  int getIndexInKeyFrame(KeyFrame* pKF);
//  bool isInKeyFrame(KeyFrame* pKF);

  void setBadFlag();
  bool isBad();

//  void replace(MapPoint* pMP);
//  MapPoint* getReplaced();

//  void increaseVisible(int n=1);
//  void increaseFound(int n=1);
//  float getFoundRatio();
  inline int getFound(){
      return mnFound;
  }

  void computeDistinctiveDescriptors();

//  cv::Mat getDescriptor();

  void updateNormalAndDepth();

//  float getMinDistanceInvariance();
//  float getMaxDistanceInvariance();
//  int predictScale(const float &currentDist, KeyFrame*pKF);
//  int predictScale(const float &currentDist, Frame* pF);

public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;


  static QMutex mGlobalMutex;

protected:

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     QMutex mMutexPos;
     QMutex mMutexFeatures;
     QMutex mMutexColor;
};
} // namespace MY_SLAM

#endif // MAPPOINT_HPP
