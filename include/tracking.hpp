#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <string>
#include <set>

#include <opencv2/core/core.hpp>

#include <QMutex>

#include "frame.hpp"
#include "ORBextractor.hpp"

using namespace ORB_SLAM2;

namespace MY_SLAM
{
class System;
class Viewer;
class FrameDrawer;
class MapDrawer;
class KeyFrame;
class Map;

class Tracking
{
  typedef pair<int, int> Match;
public:
  // Tracking states
  enum eTrackingState{
      SYSTEM_NOT_READY = -1,
      NO_IMAGES_YET = 0,
      NOT_INITIALIZED = 1,
      OK = 2,
      LOST = 3
  };

  eTrackingState mState;
  eTrackingState mLastProcessedState;

public:
  Tracking();
  Tracking(System *pSys, FrameDrawer *pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
           /*KeyFrameDatabase* pKFDB, */const std::string &strSettingPath, const int sensor);

  // Preprocess the input and call Track(). Extract features and performs stereo matching.
  cv::Mat grabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);
  cv::Mat grabImageMonocular(const cv::Mat &im, const double &timestamp);

  std::vector<cv::Mat> getTrajectories();
  std::vector<KeyFrame*> getAllKeyFrames();

  // Input sensor
  int mSensor;

  // Current Frame
  Frame mCurrentFrame;
  cv::Mat mImGray;

  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

protected:

  // Main tracking function. It is independent of the input sensor.
  void track();
  void estimate2D2D();

  // Map initialization for monocular
//  void MonocularInitialization();
  bool createInitialMapMonocular();

  // todo: move to class Initializer 2021-08-15
  static void decomposeEssentialMat(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);
  static void normalizeKeyPoints(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
  static void triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

  void findHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
  void findFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21);

  cv::Mat computeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
  cv::Mat computeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

  float checkHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
  float checkFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

  bool reconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
  bool reconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
  cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

  int checkRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                      const vector<Match> &vMatches12, vector<bool> &vbInliers,
                      const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

  //ORB
  ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
  ORBextractor* mpIniORBextractor;

  // System
  System* mpSystem;

  //Drawers
  Viewer* mpViewer;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;

  //Map
  Map* mpMap;

  // Calibration matrix
  cv::Mat mK;
  cv::Mat mDistCoef;
  float mbf;

  //New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two keyframes.
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  float mDepthMapFactor;

  //Current matches in frame
  int mnMatchesInliers;

  // Current Matches from Reference to Current
  vector<Match> mvMatches12;
  vector<bool> mvbMatched1;

  //Last Frame, KeyFrame and Relocalisation Info
  KeyFrame* mpLastKeyFrame;
  Frame mLastFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;

  //Motion Model
  cv::Mat mVelocity;

  //Color order (true RGB, false BGR, ignored if grayscale)
  bool mbRGB;

  std::vector<cv::Mat> mvCameraTraj;
  std::set<KeyFrame*> mspKeyFrames;
  QMutex mMutexCamera;

  // Ransac sets
  vector<vector<size_t> > mvSets;   
};
} // namespace MY_SLAM

#endif // TRACKING_HPP
