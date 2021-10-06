#ifndef FRAME_HPP
#define FRAME_HPP

#include <vector>

#include "ORBextractor.hpp"

using namespace std;
using namespace ORB_SLAM2;

namespace MY_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;

class Frame
{
public:
  Frame();
  // Copy constructor
  Frame(const Frame &frame);
  // Constructor for stereo camera
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor* extractorLeft, ORBextractor* extractorRight,
        cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
  // Constructor for monocular camera
  Frame(const cv::Mat &imGray, const double &timeStamp,
        ORBextractor* extractor,
        cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

//  static int descriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Extract ORB features
  void extractORB(int flag, const cv::Mat &im);
  // Set the camera pose.
  void setPose(cv::Mat Tcw);
  // Computes rotation, translation and camera center matrices from the camera pose.
  void updatePoseMatrices();
  // Returns the camera center.
  inline cv::Mat getCameraCenter(){
      return mOw.clone();
  }
  // Returns inverse of rotation
  inline cv::Mat getRotationInverse(){
      return mRwc.clone();
  }
  // Compute the cell of a keypoint (return false if outside the grid)
  bool posInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  vector<size_t> getFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
  // Search a match for each keypoint in the left image to a keypoint in the right image.
  // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
  void computeStereoMatches();


  // Feature extractor. The right is used only in the stereo case.
  ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

  // Frame timestamp.
  double mTimeStamp;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
  // In the stereo case, mvKeysUn is redundant as images must be rectified.
  // In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints, NULL pointer if no association.
  std::vector<MapPoint*> mvpMapPoints;

  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Camera pose. Transformation from world coordinate to camera coordinate
  cv::Mat mTcw;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  void undistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the constructor).
  void computeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void assignFeaturesToGrid();

  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  cv::Mat mRwc;
  cv::Mat mOw; //==mtwc
};
} // namespace MY_SLAM

#endif // FRAME_HPP
