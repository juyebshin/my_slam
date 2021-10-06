#ifndef MAPDRAWER_HPP
#define MAPDRAWER_HPP

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <QMutex>

namespace MY_SLAM
{
class Tracking;
class KeyFrame;
class Map;

class MapDrawer
{
public:
  MapDrawer(Map* pMap);
  MapDrawer(Map* pMap, const std::string &strSettingPath);
    
  Map* mpMap;

  void update(Tracking *pTracker);
  void getCurrentOpenGLCameraMatrix(float *m);
  void getCameraTrajectories(std::vector<cv::Mat> &vTraj);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;
    std::vector<cv::Mat> mvCameraTraj;
    std::vector<KeyFrame*> mvpKeyFrames;

    QMutex mMutexCamera;
};
} // namespace MY_SLAM

#endif // MAPDRAWER_HPP
