#include "keyframe.hpp"
#include "frame.hpp"
#include "ORBmatcher.hpp"
#include "map.hpp"
#include "mappoint.hpp"

#include <iostream>
using namespace std;

namespace MY_SLAM
{

long unsigned int KeyFrame::nNextId=0;

//KeyFrame::KeyFrame()
//{

//}

KeyFrame::KeyFrame(Frame &F, Map* pMap)
  : mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS),
    mnGridRows(FRAME_GRID_ROWS), mfGridElementWidthInv(F.mfGridElementWidthInv),
    mfGridElementHeightInv(F.mfGridElementHeightInv), mnTrackReferenceForFrame(0),
    mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnLoopQuery(0),
    mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), fx(F.fx), fy(F.fy),
    cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy), mbf(F.mbf), mb(F.mb),
    mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor), mfLogScaleFactor(F.mfLogScaleFactor),
    mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY),
    mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints),
    mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
  mnId=nNextId++;

  mGrid.resize(mnGridCols);
  for(int i=0; i<mnGridCols;i++)
  {
      mGrid[i].resize(mnGridRows);
      for(int j=0; j<mnGridRows; j++)
          mGrid[i][j] = F.mGrid[i][j];
  }

  setPose(F.mTcw);
}

// Pose functions
void KeyFrame::setPose(const cv::Mat &Tcw_)
{
  QMutexLocker locker(&mMutexPose);
  Tcw_.copyTo(Tcw);
  cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
  cv::Mat tcw = Tcw.rowRange(0,3).col(3);
  cv::Mat Rwc = Rcw.t();
  Ow = -Rwc*tcw;

  Twc = cv::Mat::eye(4,4,Tcw.type());
  Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
  Ow.copyTo(Twc.rowRange(0,3).col(3));
  cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
  Cw = Twc*center;
}

cv::Mat KeyFrame::getPose()
{
  QMutexLocker locker(&mMutexPose);
  return Tcw.clone();
}

cv::Mat KeyFrame::getPoseInverse()
{
  QMutexLocker locker(&mMutexPose);
  return Twc.clone();
}

cv::Mat KeyFrame::getCameraCenter()
{
  QMutexLocker locker(&mMutexPose);
  return Ow.clone();
}

cv::Mat KeyFrame::getStereoCenter()
{
  QMutexLocker locker(&mMutexPose);
  return Cw.clone();
}

cv::Mat KeyFrame::getRotation()
{
  QMutexLocker locker(&mMutexPose);
  return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::getTranslation()
{
  QMutexLocker locker(&mMutexPose);
  return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::addMapPoint(MapPoint* pMP, const size_t &idx)
{
  QMutexLocker locker(&mMutexFeatures);
  mvpMapPoints[idx] = pMP;
}

std::vector<MapPoint*> KeyFrame::getMapPointMatches()
{
  QMutexLocker locker(&mMutexFeatures);
  return mvpMapPoints;
}

int KeyFrame::trackedMapPoints(const int &minObs)
{
  QMutexLocker locker(&mMutexFeatures);

  int nPoints=0;
  const bool bCheckObs = minObs>0;
  for(int i=0; i<N; i++)
  {
      MapPoint* pMP = mvpMapPoints[i];
      if(pMP)
      {
          if(!pMP->isBad())
          {
              if(bCheckObs)
              {
                  if(mvpMapPoints[i]->observations()>=minObs)
                      nPoints++;
              }
              else
                  nPoints++;
          }
      }
  }

  return nPoints;
}

// KeyPoint functions
std::vector<size_t> KeyFrame::getFeaturesInArea(const float &x, const float  &y, const float  &r) const
{
  vector<size_t> vIndices;
  vIndices.reserve(N);

  const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
  if(nMinCellX>=mnGridCols)
      return vIndices;

  const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
  if(nMaxCellX<0)
      return vIndices;

  const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
  if(nMinCellY>=mnGridRows)
      return vIndices;

  const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
  if(nMaxCellY<0)
      return vIndices;

  for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
  {
      for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
      {
          const vector<size_t> vCell = mGrid[ix][iy];
          for(size_t j=0, jend=vCell.size(); j<jend; j++)
          {
              const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
              const float distx = kpUn.pt.x-x;
              const float disty = kpUn.pt.y-y;

              if(fabs(distx)<r && fabs(disty)<r)
                  vIndices.push_back(vCell[j]);
          }
      }
  }

  return vIndices;
}

cv::Mat KeyFrame::unprojectStereo(int i)
{
  const float z = mvDepth[i];
  if(z>0)
  {
      const float u = mvKeys[i].pt.x;
      const float v = mvKeys[i].pt.y;
      const float x = (u-cx)*z*invfx;
      const float y = (v-cy)*z*invfy;
      cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

      QMutexLocker locker(&mMutexPose);
      return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
  }
  else
      return cv::Mat();
}


// Image
bool KeyFrame::isInImage(const float &x, const float &y) const
{
  return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

void KeyFrame::setBadFlag()
{

}

bool KeyFrame::isBad()
{
  QMutexLocker locker(&mMutexConnections);
  return mbBad;
}

// Compute Scene Depth (q=2 median). Used in monocular.
float KeyFrame::computeSceneMedianDepth(const int q)
{
  // cout << "computeSceneMedianDepth" << endl;
  vector<MapPoint*> vpMapPoints;
  cv::Mat Tcw_;
  {
      QMutexLocker locker(&mMutexFeatures);
      QMutexLocker locker2(&mMutexPose);
      vpMapPoints = mvpMapPoints;
      Tcw_ = Tcw.clone();
  }

  // cout << "Tcw: " << Tcw_ << endl;

  vector<float> vDepths;
  vDepths.reserve(N);
  cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw_.at<float>(2,3);
  for(int i=0; i<N; i++)
  {
      if(mvpMapPoints[i])
      {
          MapPoint* pMP = mvpMapPoints[i];
          cv::Mat x3Dw = pMP->getWorldPos();
          float z = Rcw2.dot(x3Dw)+zcw;
          vDepths.push_back(z);
      }
  }

  sort(vDepths.begin(),vDepths.end());

  return vDepths[(vDepths.size()-1)/q];
}


} // namespace MY_SLAM
