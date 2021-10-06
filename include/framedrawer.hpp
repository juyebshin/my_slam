#ifndef FRAMEDRAWER_HPP
#define FRAMEDRAWER_HPP

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <QMutex>

namespace MY_SLAM
{
class Tracking;
class Viewer;

class FrameDrawer
{
public:
  FrameDrawer();

  void update(Tracking *pTracker);

  cv::Mat drawFrame();

protected:

  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

  cv::Mat mIm;
  int N;
  std::vector<cv::KeyPoint> mvCurrentKeys;
  std::vector<bool> mvbMap, mvbVO;
  bool mbOnlyTracking;
  int mnTracked, mnTrackedVO;
  std::vector<cv::KeyPoint> mvIniKeys;
  std::vector<int> mvIniMatches;
  int mState;

  QMutex mMutex;
};
} // namespace MY_SLAM

#endif // FRAMEDRAWER_HPP
