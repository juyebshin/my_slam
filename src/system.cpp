#include "system.hpp"
#include "viewer.hpp"
#include "window.hpp"
#include "framedrawer.hpp"
#include "tracking.hpp"
#include "mapdrawer.hpp"
#include "keyframe.hpp"
#include "map.hpp"

#include <QApplication>
#include <QDesktopWidget>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace MY_SLAM
{
System::System()
{

}

System::System(const string &strSettingsFile, const std::string &strPathToSequence, const eSensor sensor)
  : mSensor(sensor),
    mbFinishRequested(false),
    mbFinished(true)
{
  //Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  if(mSensor == MONOCULAR)
    loadMonocularImages(strPathToSequence);
  else if(mSensor == STEREO)
    loadStereoImages(strPathToSequence);
  else
  {
    cerr << "Invalid sensor type: " << static_cast<int>(mSensor) << endl;
    exit(-1);
  }

  mpMap = new Map();

  mpFrameDrawer = new FrameDrawer;
  mpMapDrawer = new MapDrawer(mpMap);

  mpTracker = new Tracking(this, mpFrameDrawer, mpMapDrawer, mpMap, strSettingsFile, mSensor);
  cout << "Tracker created" << endl;

  mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, strSettingsFile);
  cout << "Viewer created" << endl;

  mImGray = cv::Mat(376, 1241, CV_8UC3, cv::Scalar(255, 255, 255));
}

System::~System()
{
//  delete mpViewer;
}

bool System::init()
{
  mpViewer->init();
  cout << "Viewer started" << endl;

  start(TimeCriticalPriority); // TimeCriticalPriority
  cout << "System started" << endl;
  return true;
}

void System::run()
{
  mbFinished = false;

//  while(1)
//  {
    const int nImages = mvStrImageLeft.size();

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat imLeft, imRight;

    for(int ni = 0; ni < nImages; ni++)
    {
      imLeft = cv::imread(mvStrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
      if(mSensor == STEREO)
        imRight = cv::imread(mvStrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
      double tframe = mvTimestamps[ni];

      if(imLeft.empty())
      {
          cerr << endl << "Failed to load image at: "
               << string(mvStrImageLeft[ni]) << endl;
          exit(-1);
      }

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      // Track
      if(mSensor == MONOCULAR)
        trackMonocular(imLeft, tframe);
      else if(mSensor == STEREO)
        trackStereo(imLeft, imRight, tframe);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      vTimesTrack[ni] = ttrack;

      double T = 0;
      if(ni<nImages-1)
          T = mvTimestamps[ni+1]-tframe;
      else if(ni>0)
          T = tframe-mvTimestamps[ni-1];

//      cout << "time: " << ttrack << ", T: " << T << endl;

      if(ttrack<T)
          QThread::usleep((T-ttrack)*1e6);

      if(checkFinish())
        break;
    }


//    if(checkFinish())
//      break;
//  }
  // When the above iteration has reached at the end
//  if(!checkFinish())
//    Q_EMIT destroyWindow();
//  else // else wait for finish request
//  {
    while(1)
    {
      if(checkFinish())
        break;
    }
//  }

  setFinish();

  cout << "system thread finished" << endl;
}

void System::saveTrajectory(const std::string &filename)
{
  vector<KeyFrame*> vpKFs = mpMap->getAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
//  cv::Mat Two = vpKFs[0]->getPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

//  vector<KeyFrame*>::iterator lKit = vpKFs.begin();
  for(vector<KeyFrame*>::iterator lit = vpKFs.begin(), lend = vpKFs.end(); lit != lend; lit++)
  {
    KeyFrame *pKF = *lit;

    cv::Mat Tcw = pKF->getPose();
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
         Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
         Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
  }
  f.close();
  cout << endl << "trajectory saved!" << endl;
}

cv::Mat System::drawFrame()
{
  cv::Mat im;

  {
    QMutexLocker locker(&mqMutex);
    mImGray.copyTo(im);
  }

  return im;
}

void System::shutdown()
{
  requestFinish();
  while(!isFinished())
    QThread::usleep(5000);

  mpViewer->requestFinish();
  while(!mpViewer->isFinished())
    QThread::usleep(5000);

  quit();
}

void System::loadStereoImages(const std::string &strPathToSequence)
{
  ifstream fTimes;
      string strPathTimeFile = strPathToSequence + "/times.txt";
      fTimes.open(strPathTimeFile.c_str());
      while(!fTimes.eof())
      {
          string s;
          getline(fTimes,s);
          if(!s.empty())
          {
              stringstream ss;
              ss << s;
              double t;
              ss >> t;
              mvTimestamps.push_back(t);
          }
      }

      string strPrefixLeft = strPathToSequence + "/image_0/";
      string strPrefixRight = strPathToSequence + "/image_1/";

      const int nTimes = mvTimestamps.size();
      mvStrImageLeft.resize(nTimes);
      mvStrImageRight.resize(nTimes);

      for(int i=0; i<nTimes; i++)
      {
          stringstream ss;
          ss << setfill('0') << setw(6) << i;
          mvStrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
          mvStrImageRight[i] = strPrefixRight + ss.str() + ".png";
      }
}

void System::loadMonocularImages(const std::string &strPathToSequence)
{
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while(!fTimes.eof())
  {
      string s;
      getline(fTimes,s);
      if(!s.empty())
      {
          stringstream ss;
          ss << s;
          double t;
          ss >> t;
          mvTimestamps.push_back(t);
      }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";

  const int nTimes = mvTimestamps.size();
  mvStrImageLeft.resize(nTimes);

  for(int i=0; i<nTimes; i++)
  {
      stringstream ss;
      // Image file name: 000000.png
      ss << setfill('0') << setw(6) << i;
      mvStrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
  }
}

cv::Mat System::trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
  if(mSensor != STEREO)
  {
    cerr << "ERROR: you called trackStereo but input sensor was not set to STEREO." << endl;
    exit(-1);
  }

  cv::Mat Tcw = mpTracker->grabImageStereo(imLeft, imRight, timestamp);

  return Tcw;
}

cv::Mat System::trackMonocular(const cv::Mat &im, const double &timestamp)
{
  if(mSensor != MONOCULAR)
  {
    cerr << "ERROR: you called trackMonocular but input sensor was not set to MONOCULAR." << endl;
    exit(-1);
  }

  cv::Mat Tcw = mpTracker->grabImageMonocular(im, timestamp);

  return Tcw;
}

bool System::requestFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  mbFinishRequested = true;
}

bool System::isFinished()
{
  QMutexLocker locker(&mqMutexFinish);
  return mbFinished;
}

bool System::checkFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  return mbFinishRequested;
}

void System::setFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  mbFinished = true;
}
} // namespace MY_SLAM
