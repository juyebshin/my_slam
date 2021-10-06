#include "mappoint.hpp"
#include "ORBmatcher.hpp"
#include "map.hpp"
#include "keyframe.hpp"
#include "frame.hpp"

#include <iostream>
using namespace std;

namespace MY_SLAM
{
long unsigned int MapPoint::nNextId = 0;
QMutex MapPoint::mGlobalMutex;
//MapPoint::MapPoint()
//{

//}
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap)
  :
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
  Pos.copyTo(mWorldPos);
  // Added 2021-06-09 00:54

  mNormalVector = cv::Mat::zeros(3,1,CV_32F);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  QMutexLocker locker(&mpMap->mMutexPointCreation);
  mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF)
  :
  mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
  mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
  mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
  mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
  Pos.copyTo(mWorldPos);
  cv::Mat Ow = pFrame->getCameraCenter();
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector/cv::norm(mNormalVector);

  cv::Mat PC = Pos - Ow;
  const float dist = cv::norm(PC);
  const int level = pFrame->mvKeysUn[idxF].octave;
  const float levelScaleFactor =  pFrame->mvScaleFactors[level];
  const int nLevels = pFrame->mnScaleLevels;

  mfMaxDistance = dist*levelScaleFactor;
  mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  QMutexLocker locker(&mpMap->mMutexPointCreation);
  mnId=nNextId++;
}

void MapPoint::setWorldPos(const cv::Mat &Pos)
{
    QMutexLocker locker2(&mGlobalMutex);
    QMutexLocker locker(&mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::getWorldPos()
{
    QMutexLocker locker(&mMutexPos);
    return mWorldPos.clone();
}

int MapPoint::observations()
{
  QMutexLocker locker(&mMutexFeatures);
  return nObs;
}

void MapPoint::addObservation(KeyFrame* pKF,size_t idx)
{
  QMutexLocker locker(&mMutexFeatures);
  if(mObservations.count(pKF))
      return;
  mObservations[pKF]=idx;

  if(pKF->mvuRight[idx]>=0)
      nObs+=2;
  else
      nObs++;
}

void MapPoint::setBadFlag()
{

}

bool MapPoint::isBad()
{
  QMutexLocker locker(&mMutexFeatures);
  QMutexLocker locker2(&mMutexPos);
  return mbBad;
}

void MapPoint::computeDistinctiveDescriptors()
{
  // Retrieve all observed descriptors
  vector<cv::Mat> vDescriptors;

  map<KeyFrame*,size_t> observations;

  {
      QMutexLocker locker(&mMutexFeatures);
      if(mbBad)
          return;
      observations=mObservations;
  }

  if(observations.empty())
      return;

  vDescriptors.reserve(observations.size());

  for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
  {
      KeyFrame* pKF = mit->first;

      if(!pKF->isBad())
          vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
  }

  if(vDescriptors.empty())
      return;

  // Compute distances between them
  const size_t N = vDescriptors.size();

  float Distances[N][N];
  for(size_t i=0;i<N;i++)
  {
      Distances[i][i]=0;
      for(size_t j=i+1;j<N;j++)
      {
          int distij = ORBmatcher::descriptorDistance(vDescriptors[i],vDescriptors[j]);
          Distances[i][j]=distij;
          Distances[j][i]=distij;
      }
  }

  // Take the descriptor with least median distance to the rest
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for(size_t i=0;i<N;i++)
  {
      vector<int> vDists(Distances[i],Distances[i]+N);
      sort(vDists.begin(),vDists.end());
      int median = vDists[0.5*(N-1)];

      if(median<BestMedian)
      {
          BestMedian = median;
          BestIdx = i;
      }
  }

  {
      QMutexLocker locker(&mMutexFeatures);
      mDescriptor = vDescriptors[BestIdx].clone();
  }
}

void MapPoint::updateNormalAndDepth()
{
  map<KeyFrame*,size_t> observations;
  KeyFrame* pRefKF;
  cv::Mat Pos;
  {
      QMutexLocker locker(&mMutexFeatures);
      QMutexLocker locker2(&mMutexPos);
      if(mbBad)
          return;
      observations=mObservations;
      pRefKF=mpRefKF;
      Pos = mWorldPos.clone();
  }

  if(observations.empty())
      return;

  cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
  int n=0;
  for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
  {
      KeyFrame* pKF = mit->first;
      cv::Mat Owi = pKF->getCameraCenter();
      cv::Mat normali = mWorldPos - Owi;
      normal = normal + normali/cv::norm(normali);
      n++;
  }

  cv::Mat PC = Pos - pRefKF->getCameraCenter(); // pKFcur
  const float dist = cv::norm(PC);
  const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
  const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
  const int nLevels = pRefKF->mnScaleLevels;

  {
      QMutexLocker locker3(&mMutexPos);
      mfMaxDistance = dist*levelScaleFactor;
      mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
      mNormalVector = normal/n;
  }
}
} // namespace MY_SLAM
