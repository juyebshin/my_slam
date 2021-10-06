#include "framedrawer.hpp"
#include "tracking.hpp"

#include <sstream>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace MY_SLAM
{
FrameDrawer::FrameDrawer()
{
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(376,1241,CV_8UC3, cv::Scalar(0,0,0));
}

void FrameDrawer::update(Tracking *pTracker)
{
  QMutexLocker locker(&mMutex);
  pTracker->mImGray.copyTo(mIm);
  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
//  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  N = mvCurrentKeys.size();
  mvbVO = std::vector<bool>(N, false);

  mvIniKeys = pTracker->mInitialFrame.mvKeys;
  mvIniMatches = pTracker->mvIniMatches;

  for(int i = 0; i < N; i++)
  {
    if(!pTracker->mCurrentFrame.mvbOutlier[i])
    {
      mvbVO[i] = true;
    }
  }

  mState = static_cast<int>(pTracker->mLastProcessedState);
}

cv::Mat FrameDrawer::drawFrame()
{
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
  vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  vector<int> vMatches; // Initialization: correspondeces with reference keypoints
  vector<bool> vbVO;
  int state; // Tracking state

  {
    QMutexLocker locker(&mMutex);
    state = mState;

    mIm.copyTo(im);

    vIniKeys = mvIniKeys;
    vCurrentKeys = mvCurrentKeys;
    vMatches = mvIniMatches;
    vbVO = mvbVO;
  }

  if(im.channels() < 3) //this should be always true
      cv::cvtColor(im, im, CV_GRAY2BGR);

  mnTracked = 0;
  mnTrackedVO = 0;
  const float r = 5;
  const int n = vMatches.size();
  for(int i = 0; i < n; i++)
  {
    if(vMatches[i] >= 0)
    {
      cv::Point2f pt1, pt2;
      pt1.x = vCurrentKeys[vMatches[i]].pt.x - r;
      pt1.y = vCurrentKeys[vMatches[i]].pt.y - r;
      pt2.x = vCurrentKeys[vMatches[i]].pt.x + r;
      pt2.y = vCurrentKeys[vMatches[i]].pt.y + r;

      if(vbVO[vMatches[i]]) // inliers
      {
        cv::rectangle(im, pt1, pt2, cv::Scalar(0, 0, 255));
        cv::circle(im, vCurrentKeys[vMatches[i]].pt, 2, cv::Scalar(0,0,255), -1);
//        if(state == Tracking::NOT_INITIALIZED)
          cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0,0,255));
        mnTrackedVO++;
      }
      else
      {
        cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
        cv::circle(im, vCurrentKeys[vMatches[i]].pt, 2, cv::Scalar(0,255,0), -1);
//        if(state == Tracking::NOT_INITIALIZED)
          cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0,255,0));
      }

      mnTracked++;
    }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
  stringstream ss;
  if(nState==Tracking::NO_IMAGES_YET)
    ss << " WAITING FOR IMAGES ";
//  else if(nState==Tracking::NOT_INITIALIZED)
//    ss << " TRYING TO INITIALIZE | ";
//  else if(nState==Tracking::OK)
//    ss << "OK | ";
//  else if(nState==Tracking::LOST)
//  {
//    ss << " TRACK LOST. TRYING TO RELOCALIZE ";
//  }
//  else if(nState==Tracking::SYSTEM_NOT_READY)
//  {
//    ss << " LOADING ORB VOCABULARY. PLEASE WAIT...";
//  }
  else
    ss << "VO matches(green): " << mnTracked << " inliers(red): " << mnTrackedVO;

  int baseline = 0;
  cv::Size textSize = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows+textSize.height+10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height+10, im.cols, im.type());
  cv::putText(imText, ss.str(), cv::Point(5, imText.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);
}
} // namespace MY_SLAM
