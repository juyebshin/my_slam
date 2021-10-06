#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <opencv2/core/core.hpp>

#include <QMutex>

namespace MY_SLAM
{
class Frame;
class Map;
class MapPoint;
class Frame;

class KeyFrame
{
public:
//    KeyFrame();
    KeyFrame(Frame &F, Map* pMap);

    // Pose functions
    void setPose(const cv::Mat &Tcw_);
    cv::Mat getPose();
    cv::Mat getPoseInverse();
    cv::Mat getCameraCenter();
    cv::Mat getStereoCenter();
    cv::Mat getRotation();
    cv::Mat getTranslation();

    // MapPoint observation functions
    void addMapPoint(MapPoint* pMP, const size_t &idx);
    std::vector<MapPoint*> getMapPointMatches();
    int trackedMapPoints(const int &minObs);

    // KeyPoint functions
    std::vector<size_t> getFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat unprojectStereo(int i);

    // Image
    bool isInImage(const float &x, const float &y) const;

    // Set/check bad flag
    void setBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float computeSceneMedianDepth(const int q);

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }
    
    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;
    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    QMutex mMutexPose;
    QMutex mMutexConnections;
    QMutex mMutexFeatures;
};
} // namespace MY_SLAM

#endif // KEYFRAME_HPP
