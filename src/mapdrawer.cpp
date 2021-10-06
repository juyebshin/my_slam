#include "mapdrawer.hpp"
#include "tracking.hpp"
#include "keyframe.hpp"
#include "map.hpp"

#include <iostream>

using namespace std;

namespace MY_SLAM
{
MapDrawer::MapDrawer(Map* pMap) : mpMap(pMap)
{

}

MapDrawer::MapDrawer(Map* pMap, const std::string &strSettingPath) : mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
}

// this is not used
void MapDrawer::update(Tracking *pTracker)
{
    QMutexLocker locker(&mMutexCamera);
    mvCameraTraj = pTracker->getTrajectories();
//    if(mvCameraTraj.size())
//        mCameraPose = cv::Mat(4, 4, CV_32F, mvCameraTraj.back());
    mvpKeyFrames = pTracker->getAllKeyFrames();
}

void MapDrawer::getCurrentOpenGLCameraMatrix(float *m)
{
    m = new float[16];
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            QMutexLocker locker(&mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        m[0] = Rwc.at<float>(0,0);
        m[1] = Rwc.at<float>(1,0);
        m[2] = Rwc.at<float>(2,0);
        m[3]  = 0.0;

        m[4] = Rwc.at<float>(0,1);
        m[5] = Rwc.at<float>(1,1);
        m[6] = Rwc.at<float>(2,1);
        m[7]  = 0.0;

        m[8] = Rwc.at<float>(0,2);
        m[9] = Rwc.at<float>(1,2);
        m[10] = Rwc.at<float>(2,2);
        m[11]  = 0.0;

        m[12] = twc.at<float>(0);
        m[13] = twc.at<float>(1);
        m[14] = twc.at<float>(2);
        m[15]  = 1.0;
    }
    else
    {
        // Identity matrix
        m[0] = 1.0;
        m[1] = 0.0;
        m[2] = 0.0;
        m[3]  = 0.0;

        m[4] = 0.0;
        m[5] = 1.0;
        m[6] = 0.0;
        m[7]  = 0.0;

        m[8] = 0.0;
        m[9] = 0.0;
        m[10] = 1.0;
        m[11]  = 0.0;

        m[12] = 0.0;
        m[13] = 0.0;
        m[14] = 0.0;
        m[15]  = 1.0;
    }
}

void MapDrawer::getCameraTrajectories(std::vector<cv::Mat> &vTraj)
{
    QMutexLocker locker(&mMutexCamera);
    // int N = mvpKeyFrames.size();
    // sort(mvpKeyFrames.begin(), mvpKeyFrames.end(), KeyFrame::lId);
    // vTraj.resize(mvpKeyFrames.size()); // reserve does not work!
    // for(int i = 0; i < N; i++)
    // {
    //   KeyFrame* pKF = mvpKeyFrames[i];
    //   vTraj[i] = pKF->getPose();
    // }
    std::vector<KeyFrame*> vpKFs = mpMap->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
    vTraj.resize(vpKFs.size());
    if(vpKFs.size())
      cout << "last keyframe id: " << vpKFs.back()->mnId << endl;
    for(size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        vTraj[i] = pKF->getPose();
    }
}
} // namespace MY_SLAM
