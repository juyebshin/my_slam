#include "tracking.hpp"
#include "system.hpp"
#include "framedrawer.hpp"
#include "ORBmatcher.hpp"
#include "mapdrawer.hpp"
#include "keyframe.hpp"
#include "map.hpp"
#include "mappoint.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// thirdparty
#include "Thirdparty/DBoW2/DUtils/Random.h"

using namespace std;

namespace MY_SLAM
{
Tracking::Tracking()
{

}

Tracking::Tracking(System *pSys, FrameDrawer *pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
         /*KeyFrameDatabase* pKFDB, */const std::string &strSettingPath, const int sensor)
  :
    mState(NO_IMAGES_YET), mSensor(sensor), mpSystem(pSys), mnLastRelocFrameId(0),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap)
{
  // Load camera parameters from settings file

  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3,3,CV_32F);
  K.at<float>(0,0) = fx;
  K.at<float>(1,1) = fy;
  K.at<float>(0,2) = cx;
  K.at<float>(1,2) = cy;
  K.copyTo(mK);
  /** mK = [
   * [fx, 0, cx];
   * [0, fy, cy];
   * [0,  0,  1];
   * */

  cv::Mat DistCoef(4,1,CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if(k3!=0)
  {
      DistCoef.resize(5);
      DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  float fps = fSettings["Camera.fps"];
  if(fps==0)
      fps=30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  cout << endl << "Camera Parameters: " << endl;
  cout << "- fx: " << fx << endl;
  cout << "- fy: " << fy << endl;
  cout << "- cx: " << cx << endl;
  cout << "- cy: " << cy << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if(DistCoef.rows==5)
      cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;
  cout << "- fps: " << fps << endl;

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if(mbRGB)
      cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
      cout << "- color order: BGR (ignored if grayscale)" << endl;

  // Load ORB parameters

  int nFeatures = fSettings["ORBextractor.nFeatures"];
  float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
  int nLevels = fSettings["ORBextractor.nLevels"];
  int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
  int fMinThFAST = fSettings["ORBextractor.minThFAST"];


  if(sensor == System::STEREO)
  {
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
  }
  else if(sensor == System::MONOCULAR)
    mpORBextractorLeft = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

  cout << endl  << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

  if(sensor==System::STEREO/* || sensor==System::RGBD*/)
  {
      mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
      cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
  }

//  if(sensor==System::RGBD)
//  {
//      mDepthMapFactor = fSettings["DepthMapFactor"];
//      if(fabs(mDepthMapFactor)<1e-5)
//          mDepthMapFactor=1;
//      else
//          mDepthMapFactor = 1.0f/mDepthMapFactor;
//  }
}

cv::Mat Tracking::grabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;

  if(mImGray.channels()==3)
  {
    if(mbRGB)
    {
      cv::cvtColor(mImGray,mImGray,CV_RGB2GRAY);
      cv::cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
    }
    else
    {
      cv::cvtColor(mImGray,mImGray,CV_BGR2GRAY);
      cv::cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
    }
  }
  else if(mImGray.channels()==4)
  {
    if(mbRGB)
    {
      cv::cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
      cv::cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
    }
    else
    {
      cv::cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
      cv::cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
    }
  }

  mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef, mbf, mThDepth);

  // Track();
  // Set Frame pose to the origin
  mCurrentFrame.setPose(cv::Mat::eye(4, 4, CV_32F));

  mpFrameDrawer->update(this);

  return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::grabImageMonocular(const cv::Mat &im, const double &timestamp)
{
  mImGray = im;

  if(mImGray.channels()==3)
  {
      if(mbRGB)
          cvtColor(mImGray,mImGray,CV_RGB2GRAY);
      else
          cvtColor(mImGray,mImGray,CV_BGR2GRAY);
  }
  else if(mImGray.channels()==4)
  {
      if(mbRGB)
          cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
      else
          cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
  }

  mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);

  // Track();
  // Set Frame pose to the origin
//  mCurrentFrame.setPose(cv::Mat::eye(4, 4, CV_32F));
  track();

  return mCurrentFrame.mTcw.clone();
}

std::vector<cv::Mat> Tracking::getTrajectories()
{
    QMutexLocker locker(&mMutexCamera);
    return vector<cv::Mat>(mvCameraTraj.begin(), mvCameraTraj.end());
}

std::vector<KeyFrame*> Tracking::getAllKeyFrames()
{
  QMutexLocker locker(&mMutexCamera);
  vector<KeyFrame*> vpKeyFrames = mpMap->getAllKeyFrames();
  return vector<KeyFrame*>(vpKeyFrames.begin(), vpKeyFrames.end());
}

void Tracking::track()
{
  if(mState == NO_IMAGES_YET)
  {
    mCurrentFrame.setPose(cv::Mat::eye(4, 4, CV_32F)); // todo 2021-08-12
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  estimate2D2D();

  // cout << "initial frame tracked: " << mInitialFrame.mTcw << endl;
  // cout << "current frame tracked: " << mCurrentFrame.mTcw << endl;
  // cout << "tracked trajectoriy memories: "
  //      << mvCameraTraj.size()*sizeof(mvCameraTraj[0])*sizeof(vector<cv::Mat>) << endl;
//  QMutexLocker locker(&mMutexCamera);
//  cout << "tracked trajectories: " << mvCameraTraj.size() << endl;
//  for(int i = 0; i < mvCameraTraj.size(); i++)
//  {
//    cout << "pointer to mvCameraTraj[i]: " << (int*)mvCameraTraj[i] << endl;
//    cv::Mat Tcw(4, 4, CV_32F);
//    memcpy(Tcw.data, mvCameraTraj[i], sizeof(float)*4*4);
//    cout << "Tcw " << (int)i << ": " << Tcw << endl;
//  }

  mpFrameDrawer->update(this);
  // mpMapDrawer->update(this);
  cout << endl;
}

void Tracking::estimate2D2D()
{
  if(mState == NOT_INITIALIZED)
  {
    mCurrentFrame.setPose(cv::Mat::eye(4, 4, CV_32F));
    mInitialFrame = Frame(mCurrentFrame);
    mLastFrame = Frame(mCurrentFrame);
    mpLastKeyFrame = 0;

    if((int)mCurrentFrame.mvKeys.size() > 100)
    {
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
          mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

      fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
      mState = OK;
    }
    {      
      QMutexLocker locker(&mMutexCamera);
      mvCameraTraj.clear();
//      mvCameraTraj.push_back(mInitialFrame.mTcw);

      mspKeyFrames.clear();
      KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap);
//      mspKeyFrames.insert(pKFini);
      mpMap->addKeyFrame(pKFini);
    }
  }
  else
  {
    if((int)mCurrentFrame.mvKeys.size() <= 100)
    {
      cout << "not enough keypoints" << endl;
      mState = NOT_INITIALIZED;
      return;
    }

    mInitialFrame = Frame(mLastFrame);
    ORB_SLAM2::ORBmatcher matcher(0.9, true);
    int nmatches = matcher.searchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);


    if(nmatches < 20)
    {
      cout << "not enough matches: " << nmatches << endl;
//      mState = NOT_INITIALIZED;
      static int fail = 0;
      if(fail++ > 10)
      {
        cout << "Tracking failed. Reset..." << endl;
        mState = NOT_INITIALIZED;
      }
      return;
    }

    // cout << "mInitialFrame.mnID: " << mInitialFrame.mnId << endl;
    cout << "mCurrentFrame.mnID: " << mCurrentFrame.mnId << endl;

    // todo: move to class Initializer 2021-08-15
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvMatches12.clear();
    mvMatches12.reserve(mCurrentFrame.mvKeysUn.size()); // current frame
    mvbMatched1.resize(mInitialFrame.mvKeysUn.size());
    // vMatches12: mvIniMatches
    for(size_t i=0, iend=mvIniMatches.size();i<iend; i++)
    {
        if(mvIniMatches[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,mvIniMatches[i])); // mvIniMatches in Tracking::MonocularInitialization()
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(200,vector<size_t>(8,0)); // mMaxIterations : 200

    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<200; it++) // mMaxIterations : 200
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&Tracking::findHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Tracking::findFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

//    cout << "mvMatches12.size(): " << mvMatches12.size() << endl;
//    cout << "vbMatchesInliersH.size(): " << vbMatchesInliersH.size() << endl;
//    cout << "vbMatchesInliersF.size(): " << vbMatchesInliersF.size() << endl;

    // Compute ratio of scores
    float RH = SH/(SH+SF);

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
//    mvCameraTraj.clear();
//    mvCameraTraj.push_back(mInitialFrame.mTcw);
    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH>0.40)
    {
      cout << "reconstruct H" << endl;
      if(reconstructH(vbMatchesInliersH,H,mK,Rcw,tcw,mvIniP3D,vbTriangulated,-1.0,20)) // minParallax = 1.0, minGood = 50
      {
        int idxInlier = 0;
        for(size_t i = 0; i < mvIniMatches.size(); i++)
        {
          if(mvIniMatches[i] >= 0)
          {
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = (!vbMatchesInliersH[idxInlier++]);
            // cv::Mat pos(mvIniP3D[i]);
            // cv::Mat worldPos = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3)*pos + mCurrentFrame.mTcw.rowRange(0,3).col(3);
            // mvIniP3D[i] = cv::Point3f(worldPos.at<float>(0), worldPos.at<float>(1), worldPos.at<float>(2));
          }
        }
//        QMutexLocker locker(&mMutexCamera);
        // mvCameraTraj.clear();
        // mvCameraTraj.push_back(mInitialFrame.mTcw);
//        mvCameraTraj.push_back(mCurrentFrame.mTcw);

//        mspKeyFrames.clear();
//        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);
//        mspKeyFrames.insert(pKFcur);

        cout << "traj push_back" << endl;
      }
      else
      {
        cout << "failed to reconstruct" << endl;
        goto reconF;
//        return;
      }
    }
    else //if(pF_HF>0.6)
    {
      reconF:
      cout << "reconstruct F" << endl;
      if(reconstructF(vbMatchesInliersF,F,mK,Rcw,tcw,mvIniP3D,vbTriangulated,-1.0,20)) // minParallax = 1.0, minGood = 50
      {
        int idxInlier = 0;
        for(size_t i = 0; i < mvIniMatches.size(); i++)
        {
          if(mvIniMatches[i] >= 0)
          {
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = (!vbMatchesInliersF[idxInlier++]);
            // cv::Mat pos(mvIniP3D[i]);
            // cv::Mat worldPos = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3)*pos + mCurrentFrame.mTcw.rowRange(0,3).col(3);
            // mvIniP3D[i] = cv::Point3f(worldPos.at<float>(0), worldPos.at<float>(1), worldPos.at<float>(2));            
          }
        }
        // motion model
        // cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
        // mInitialFrame.getRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
        // mInitialFrame.getCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
//        mVelocity = mCurrentFrame.mTcw*LastTwc;

//        QMutexLocker locker(&mMutexCamera);
//         mvCameraTraj.clear();
//         mvCameraTraj.push_back(mInitialFrame.mTcw);
//        mvCameraTraj.push_back(mCurrentFrame.mTcw);

//        mspKeyFrames.clear();
//        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);
//        mspKeyFrames.insert(pKFcur);

        cout << "traj push_back" << endl;
      }
      else
      {
        cout << "failed to reconstruct" << endl;
        return;
      }
    }

//    cout << "Rcw: " << Rcw << endl;
//    cout << "tcw: " << tcw << endl;

    // relative scale computation
//    cv::Mat O2 = -Rcw.t()*tcw;
//    vector< vector<size_t> > vSets(50,vector<size_t>(2,0)); // mMaxIterations : 200
//    vector<double> vfScale(vSets.size(), 0.0);
//    for(size_t it = 0; it < vSets.size(); it++) // 50
//    {
//      vAvailableIndices = vAllIndices;
//      int pair[2] = {0, };
//      for(int i = 0; i < 2; i++)
//      {
//        int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
//        int idx = vAvailableIndices[randi];
//        if(!vbTriangulated[idx])
//        {
//          i--;
//          continue;
//        }
//        pair[i] = idx;

//        vAvailableIndices[randi] = vAvailableIndices.back();
//        vAvailableIndices.pop_back();
//      }

//      cv::Mat X1(mvIniP3D[pair[0]]);
//      cv::Mat X2(mvIniP3D[pair[1]]);

//      cv::Mat X1c2 = X1 - O2; //Rcw*X1 + tcw;
//      cv::Mat X2c2 = X2 - O2; //Rcw*X2 + tcw;

//      double dist1 = cv::norm(X1 - X2);
//      double dist2 = cv::norm(X1c2 - X2c2);

//      double r = dist1 / dist2;
//      vfScale[it] = r;

//      cout << "X1: " << X1 << endl;
//      cout << "X1c2: " << X1c2 << endl;
////      cout << "scale: " << r << endl;
//    }
//    sort(vfScale.begin(), vfScale.end());
//    cout << "median relative scale: " << vfScale[(vfScale.size()-1)/2] << endl;
//    cout << "zero scale count: " << count(vfScale.begin(), vfScale.end(), 0.0) << endl;

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(Tcw.rowRange(0,3).col(3));
    mCurrentFrame.setPose(Tcw);

    if(!createInitialMapMonocular())
      return;

    // KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);
    // mpMap->addKeyFrame(pKFcur);


//     vector<cv::Point2f> pts1;
//     vector<cv::Point2f> pts2;
//     // Reference(Initial) Frame: 1, Current Frame: 2
//     mvMatches12.clear();
//     mvMatches12.reserve(mCurrentFrame.mvKeysUn.size()); // mvKeys2
//     mvbMatched1.resize(mInitialFrame.mvKeysUn.size()); // mvKeys1
//     for(size_t i = 0, n = mvIniMatches.size(); i < n; i++)
//     {
//       if(mvIniMatches[i] >= 0)
//       {
//         pts1.push_back(mInitialFrame.mvKeysUn[i].pt);
//         pts2.push_back(mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt);
//         mvMatches12.push_back(make_pair(i, mvIniMatches[i])); // mvIniMatches in Tracking::MonocularInitialization()
//         mvbMatched1[i] = true;
//       }
//       else
//         mvbMatched1[i] = false;
//     }

//     vector<uchar> match_mask; // arg for cv::findEssentialMat
//     vector<bool> vbMatchesInliersF;

//     const int N = mvIniMatches.size();
//     vbMatchesInliersF = vector<bool>(mvMatches12.size(), false);
    
//     // F estimation
//    cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 3.0, 0.99, match_mask);
//    F.convertTo(F, CV_32F);
//    cv::Mat u,w,vt;
//    cv::SVDecomp(F, w, u, vt);
// //    cout << "F 32f: " << F << endl;
// //    cout << "singular values of F: " << w << endl;
// //    cout << "inlier size: " << count(match_mask.begin(), match_mask.end(), 1) << endl;

//     // E estimation
//     // cv::Mat E21 = cv::findEssentialMat(pts1, pts2, mK, cv::RANSAC, 0.999, 3.0, match_mask);
//     // E21.convertTo(E21, CV_32F);

//     int idxInlier = 0;
//     for(size_t i = 0; i < N; i++)
//     {
//       if(mvIniMatches[i] >= 0)
//       {
//         mCurrentFrame.mvbOutlier[mvIniMatches[i]] = static_cast<bool>(!match_mask[idxInlier]);
//         vbMatchesInliersF[idxInlier] = static_cast<bool>(match_mask[idxInlier++]);
//       }
//     }

//     int inliers = count(vbMatchesInliersF.begin(), vbMatchesInliersF.end(), true);

//     cv::Mat Rcw; // Current Camera Rotation
//     cv::Mat tcw; // Current Camera Translation
//     vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
//     cv::Mat P1 = cv::Mat::eye(4,4,CV_32F);
    

//     if( reconstructF(vbMatchesInliersF, F, mK, Rcw, tcw, mvIniP3D, vbTriangulated, 1.0, 50) )
//     {
//       Rcw.copyTo(P1.rowRange(0,3).colRange(0,3));
//       tcw.copyTo(P1.rowRange(0,3).col(3));
//       mCurrentFrame.setPose(mInitialFrame.mTcw*P1);
//       QMutexLocker locker(&mMutexCamera);
//       // mvCameraTraj.clear();
//       // mvCameraTraj.push_back(mInitialFrame.mTcw);
//       mvCameraTraj.push_back(mCurrentFrame.mTcw);

//       cout << "traj push_back" << endl;
//     }
//     else
//     {
//       cout << "failed to reconstruct" << endl;
//       return;
//     }

//    cv::Mat E21 = mK.t()*F*mK;
//    cv::Mat R1, R2, t;
//    decomposeEssentialMat(E21, R1, R2, t);

//    // four hypotheses: [R1 t1], [R1 t2], [R2 t1], [R2 t2]
//    cv::Mat t1 =  t;
//    cv::Mat t2 = -t;
//    cv::Mat P1 = cv::Mat::eye(4,4,CV_32F), P2 = cv::Mat::eye(4,4,CV_32F), P3 = cv::Mat::eye(4,4,CV_32F), P4 = cv::Mat::eye(4,4,CV_32F);

//    {
//    QMutexLocker locker(&mMutexCamera);
//    mvCameraTraj.clear();
//    mvCameraTraj.push_back(mInitialFrame.mTcw);
//    R1.copyTo(P1.rowRange(0,3).colRange(0,3));
//    t1.copyTo(P1.rowRange(0,3).col(3));
//    mvCameraTraj.push_back(P1);

//    R2.copyTo(P2.rowRange(0,3).colRange(0,3));
//    t1.copyTo(P2.rowRange(0,3).col(3));
//    mvCameraTraj.push_back(P2);

//    R1.copyTo(P3.rowRange(0,3).colRange(0,3));
//    t2.copyTo(P3.rowRange(0,3).col(3));
//    mvCameraTraj.push_back(P3);

//    R2.copyTo(P4.rowRange(0,3).colRange(0,3));
//    t2.copyTo(P4.rowRange(0,3).col(3));
//    mvCameraTraj.push_back(P4);
//    }

//    // Reconstruct with the 4 hyphoteses and check
//    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
//    vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
//    float parallax1, parallax2, parallax3, parallax4;

    mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
    for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

//    fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

    mLastFrame = Frame(mCurrentFrame);
  }
}

bool Tracking::createInitialMapMonocular()
{
  QMutexLocker locker(&mMutexCamera);
  KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap);
  KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);

//  long unsigned int nKFiniId = mpMap->getMaxKFid();
  // mpMap->addKeyFrame(pKFini);
  mpMap->addKeyFrame(pKFcur);
  cv::Mat Rc1w = (pKFini->getRotation());
  cv::Mat tc1w = (pKFini->getTranslation());

  // cout << "mInitialFrame map points: " << mInitialFrame.mvpMapPoints.size() << endl;
  // cout << "mvIniMatches: " << mvIniMatches.size() << endl;

  // Create MapPoints and asscoiate to keyframes
  for(size_t i = 0; i < mvIniMatches.size(); i++)
  {
      if(mvIniMatches[i]<0)
          continue;

      //Create MapPoint. 3 by 1
      cv::Mat worldPos(mvIniP3D[i]);
      // worldPos = Rc1w*worldPos + tc1w;

      // cv::Mat worldPos(Rc1w*cv::Mat(mvIniP3D[i]) + tc1w);

      MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

      // if(!mpLastKeyFrame)
      pKFini->addMapPoint(pMP, i);
      pKFcur->addMapPoint(pMP, mvIniMatches[i]);

      // if(!mpLastKeyFrame)
      // Add observed keyframe and keypoint index
      pMP->addObservation(pKFini,i);
      pMP->addObservation(pKFcur,mvIniMatches[i]);


      pMP->computeDistinctiveDescriptors();
      pMP->updateNormalAndDepth();

      //Fill Current Frame structure
      mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
//      mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

      //Add to Map
      mpMap->addMapPoint(pMP);
  }

  // cout << "map points added in map" << endl;

  // Set median depth to 1
  // float medianDepth = (mpLastKeyFrame ? mpLastKeyFrame->computeSceneMedianDepth(2) : pKFini->computeSceneMedianDepth(2));
  float medianDepth = 1.; //pKFcur->computeSceneMedianDepth(2); // negati
  float invMedianDepth = 1.0f/medianDepth;

  if(medianDepth <= 0 || pKFcur->trackedMapPoints(1) < 20)
  {
      cout << "Wrong initialization, not scaled..." << endl;
//      Reset();
      return false;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->getPose();
  Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*(medianDepth <= 0 ? 1 : invMedianDepth);
  pKFcur->setPose(Tc2w*mInitialFrame.mTcw); // *mInitialFrame.mTcw

  // Scale points
  // vector<MapPoint*> vpAllMapPoints = (mpLastKeyFrame ? mpLastKeyFrame->getMapPointMatches() : pKFini->getMapPointMatches());
  vector<MapPoint*> vpAllMapPoints = pKFini->getMapPointMatches();
  for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
  {
      if(vpAllMapPoints[iMP])
      {
          MapPoint* pMP = vpAllMapPoints[iMP];
          pMP->setWorldPos(pMP->getWorldPos()*invMedianDepth);
      }
  }

  // cout << "mInitialFrame Tcw: " << mInitialFrame.mTcw << endl;
  // cout << "mCurrentFrame Tcw before medianDepth: " << mCurrentFrame.mTcw << endl;
  mCurrentFrame.setPose(pKFcur->getPose()); // *mInitialFrame.mTcw
  // KeyFrame* pKFcurNew = new KeyFrame(mCurrentFrame, mpMap);
  // mpMap->addKeyFrame(pKFcurNew);
//  cout << "mCurrentFrame Tcw after medianDepth: " << mCurrentFrame.mTcw << endl;
  mpLastKeyFrame = pKFcur;

  return true;
}

void Tracking::decomposeEssentialMat(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
  cv::Mat u,w,vt;
  cv::SVDecomp(E, w, u, vt);

  // cout << "E = USV'" << endl;
  // cout << "S: " << w << endl;
  // cout << "U: " << u << endl;
  // cout << "V': " << vt << endl;

  u.col(2).copyTo(t);
  t = t/cv::norm(t);

  cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
  W.at<float>(0,1) = -1;
  W.at<float>(1,0) = 1;
  W.at<float>(2,2) = 1;

  R1 = u*W*vt;
  if(cv::determinant(R1) < 0)
    R1 = -R1;

  R2 = u*W.t()*vt;
  if(cv::determinant(R2) < 0)
    R2 = -R2;
}

void Tracking::normalizeKeyPoints(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
  float meanX = 0;
  float meanY = 0;
  const int N = vKeys.size();

  vNormalizedPoints.resize(N);

  for(int i=0; i<N; i++)
  {
      meanX += vKeys[i].pt.x;
      meanY += vKeys[i].pt.y;
  }

  meanX = meanX/N;
  meanY = meanY/N;

  float meanDevX = 0;
  float meanDevY = 0;

  for(int i=0; i<N; i++)
  {
      vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
      vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

      meanDevX += fabs(vNormalizedPoints[i].x);
      meanDevY += fabs(vNormalizedPoints[i].y); // abs: |x-x0| distance
  }

  // mean absolute distance
  meanDevX = meanDevX/N;
  meanDevY = meanDevY/N;

  float sX = 1.0/meanDevX; // scale
  float sY = 1.0/meanDevY;

  for(int i=0; i<N; i++)
  {
      vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
      vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
  }

  T = cv::Mat::eye(3,3,CV_32F);
  T.at<float>(0,0) = sX;
  T.at<float>(1,1) = sY;
  T.at<float>(0,2) = -meanX*sX;
  T.at<float>(1,2) = -meanY*sY;
}

void Tracking::triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void Tracking::findHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    normalizeKeyPoints(mInitialFrame.mvKeysUn,vPn1, T1); // mvKeys1
    normalizeKeyPoints(mCurrentFrame.mvKeysUn,vPn2, T2); // mvKeys2
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<200; it++) // mMaxIterations : 200
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = computeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = checkHomography(H21i, H12i, vbCurrentInliers, 1.0); // mSigna = 1.0

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

void Tracking::findFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    normalizeKeyPoints(mInitialFrame.mvKeysUn,vPn1, T1); // mvKeys1
    normalizeKeyPoints(mCurrentFrame.mvKeysUn,vPn2, T2); // mvKeys2
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<200; it++) // mMaxIterations = 200
    {
        // Select a minimum set
        for(int j=0; j<8; j++) // 8-point algorithm
        {
            int idx = mvSets[it][j]; // mvSets: random idx for RANSAC

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = computeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = checkFundamental(F21i, vbCurrentInliers, 1.0); // mSigma = 1.0

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

cv::Mat Tracking::computeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat Tracking::computeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}

float Tracking::checkHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mInitialFrame.mvKeysUn[mvMatches12[i].first]; // mvKeys1
        const cv::KeyPoint &kp2 = mCurrentFrame.mvKeysUn[mvMatches12[i].second]; // mvKeys2

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float Tracking::checkFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841; // reprojection error threshold chi square at 95%
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mInitialFrame.mvKeysUn[mvMatches12[i].first]; // mvKeys1 := ReferenceFrame.mvKeysUn
        const cv::KeyPoint &kp2 = mCurrentFrame.mvKeysUn[mvMatches12[i].second]; // mvKeys2 := CurrentFrame.mvKeysUn

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2; // = ax + by + c

        const float squareDist1 = num2*num2/(a2*a2+b2*b2); // distance from l2 to x2 (error)

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1; // lower the chiSquare1 the better

        // Reprojection error in first image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1); // distance from l1 to x1 (error)

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool Tracking::reconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
  int N=0;
  for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
      if(vbMatchesInliers[i])
          N++;

  // Compute Essential Matrix from Fundamental Matrix
  cv::Mat E21 = K.t()*F21*K;
  cv::Mat R1, R2, t;
  decomposeEssentialMat(E21, R1, R2, t);

  // four hypotheses: [R1 t1], [R1 t2], [R2 t1], [R2 t2]
  cv::Mat t1 =  t;
  cv::Mat t2 = -t;
  cv::Mat P1 = cv::Mat::eye(4,4,CV_32F), P2 = cv::Mat::eye(4,4,CV_32F), P3 = cv::Mat::eye(4,4,CV_32F), P4 = cv::Mat::eye(4,4,CV_32F);

//  {
//    QMutexLocker locker(&mMutexCamera);
//    mvCameraTraj.clear();
//    mvCameraTraj.push_back(cv::Mat::eye(4,4,CV_32F));
//    R1.copyTo(P1.rowRange(0,3).colRange(0,3));
//    t1.copyTo(P1.rowRange(0,3).col(3));
//    mvCameraTraj.push_back(P1);
//    // cout << "[R1 | t1]: " << P << endl;

//    R2.copyTo(P2.rowRange(0,3).colRange(0,3));
//    t1.copyTo(P2.rowRange(0,3).col(3));
//    // cout << "[R2 | t1]: " << P << endl;
//    mvCameraTraj.push_back(P2);

//    R1.copyTo(P3.rowRange(0,3).colRange(0,3));
//    t2.copyTo(P3.rowRange(0,3).col(3));
//    // cout << "[R1 | t2]: " << P << endl;
//    mvCameraTraj.push_back(P3);

//    R2.copyTo(P4.rowRange(0,3).colRange(0,3));
//    t2.copyTo(P4.rowRange(0,3).col(3));
//    // cout << "[R2 | t2]: " << P << endl;
//    mvCameraTraj.push_back(P4);
//  }

  // Reconstruct with the 4 hyphoteses and check
  vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
  vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
  float parallax1, parallax2, parallax3, parallax4;

  int nGood1 = checkRT(R1,t1,mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0, vbTriangulated1, parallax1);
  int nGood2 = checkRT(R2,t1,mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0, vbTriangulated2, parallax2);
  int nGood3 = checkRT(R1,t2,mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0, vbTriangulated3, parallax3);
  int nGood4 = checkRT(R2,t2,mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0, vbTriangulated4, parallax4);
//  cout << "nGood1: " << nGood1 << ", nGood2: " << nGood2 << ", nGood3: " << nGood3 << ", nGood4: " << nGood4 << endl;
  // todo 2021-08-11

  int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

  R21 = cv::Mat();
  t21 = cv::Mat();

  int nMinGood = max(static_cast<int>(0.3*N), minTriangulated); // 0.9

  int nsimilar = 0; // to check a clear winner
  if(nGood1 > 0.7*maxGood) // 0.7
      nsimilar++;
  if(nGood2 > 0.7*maxGood)
      nsimilar++;
  if(nGood3 > 0.7*maxGood)
      nsimilar++;
  if(nGood4 > 0.7*maxGood)
      nsimilar++;

  // If there is not a clear winner or not enough triangulated points reject initialization
  if(maxGood < nMinGood /*|| nsimilar > 1*/)
  {
      cout << "there is not a clear winner or not enough triangulated points reject initialization" << endl;
      cout << "maxGood: " << maxGood << ", nMinGood: " << nMinGood << ", nsimilar: " << nsimilar << endl;
      return false;
  }

  // If best reconstruction has enough parallax initialize
  if(maxGood == nGood1)
  {
      if(parallax1 > minParallax)
      {
          vP3D = vP3D1;
          vbTriangulated = vbTriangulated1;

          R1.copyTo(R21);
          t1.copyTo(t21);
          return true;
      }
  }else if(maxGood == nGood2)
  {
      if(parallax2>minParallax)
      {
          vP3D = vP3D2;
          vbTriangulated = vbTriangulated2;

          R2.copyTo(R21);
          t1.copyTo(t21);
          return true;
      }
  }else if(maxGood==nGood3)
  {
      if(parallax3>minParallax)
      {
          vP3D = vP3D3;
          vbTriangulated = vbTriangulated3;

          R1.copyTo(R21);
          t2.copyTo(t21);
          return true;
      }
  }else if(maxGood==nGood4)
  {
      if(parallax4>minParallax)
      {
          vP3D = vP3D4;
          vbTriangulated = vbTriangulated4;

          R2.copyTo(R21);
          t2.copyTo(t21);
          return true;
      }
  }

//  cout << "not enough parallax" << endl;
//  cout << "minParallax: " << minParallax << ", parallax1: " << parallax1  << ", parallax2: " << parallax2
//   << ", parallax3: " << parallax3 << ", parallax4: " << parallax4 << endl;

  return false;
}

bool Tracking::reconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;

        //mvKeys1: mInitialFrame.mvKeysUn, mvKeys2: mCurrentFrame.mvKeysUn, mSigna2 = 1.0
        int nGood = checkRT(vR[i],vt[i],mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*1.0, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

int Tracking::checkRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    int infin = 0;
    int depth1 = 0;
    int depth2 = 0;
    int rep1 = 0;
    int rep2 = 0;

    int cnt = 0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++) // vMatches12
    {
        if(!vbMatchesInliers[i])
            continue;

        cnt++;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        triangulate(kp1, kp2, P1, P2, p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first] = false;
            infin++;
            continue;
        }

//        if(i == 0)
//        {
//          cout << "p3dC1: " << p3dC1 << endl;
//          cout << "O1: " << O1 << endl;
//          cout << "O2: " << O2 << endl;
//        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
        {
          depth1++;
          continue;
        }

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
        {
          depth2++;
          continue;
        }

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1 > th2)
        {
          rep1++;
          continue;
        }

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
        {
          rep2++;
          continue;
        }

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

//    cout << "infin: " << infin << ", depth1: " << depth1 << ", depth2: "
//    << depth2 << ", rep1: " << rep1 << ", rep2: " << rep2 << endl;

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end()); // default: ascending order 오름차순

        size_t idx = min(50,int(vCosParallax.size()-1)); // why 50??
        // size_t idx = int(vCosParallax.size()-1); // why 50??
//        cout << "vCosParallax idx: " << idx << endl;
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}
} // namespace MY_SLAM
