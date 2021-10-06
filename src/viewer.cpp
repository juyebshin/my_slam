#include "viewer.hpp"
#include "window.hpp"
#include "imageform.hpp"
#include "system.hpp"
#include "framedrawer.hpp"
#include "glwidget.hpp"
#include "mapdrawer.hpp"
#include "keyframe.hpp"

#include <QApplication>
#include <QDesktopWidget>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <chrono>
#include <unistd.h>

#include <GL/glu.h>


using namespace std;

namespace MY_SLAM
{
Viewer::Viewer()
  :
    mbFinishRequested(false),
    mbFinished(true)
{

}

Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, const std::string &strSettingPath)
  :
    mpSystem(pSystem),
    mpFrameDrawer(pFrameDrawer),
    mpMapDrawer(pMapDrawer),
    mbFinishRequested(false),
    mbFinished(false),
    mStrSettingPath(strSettingPath)
{
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  float fps = fSettings["Camera.fps"];
  if(fps < 1)
      fps = 30;
  mT = 1e3/fps;

//  qRegisterMetaType<cv::Mat>("cv::Mat");
//  qRegisterMetaType< std::vector<cv::Mat> >("std::vector<cv::Mat>");
//  qRegisterMetaType< std::vector<KeyFrame*> >("std::vector<cv::Mat>");
  // qRegisterMetaType< std::vector<cv::Mat> >("std::vector<cv::Mat*>");
//  qRegisterMetaType< std::vector<cv::Mat> >("std::vector<uchar*>");
}

Viewer::~Viewer()
{
  if(mqWindow)
    delete mqWindow;
}

bool Viewer::init()
{
  mqWindow = new Window(mStrSettingPath);
  mqWindow->resize(1024, 768);
  mqWindow->setWindowTitle("Trajectory");
  int desktopArea = QApplication::desktop()->width() *
      QApplication::desktop()->height();
  int widgetArea = mqWindow->width() * mqWindow->height();
  if (((float)widgetArea / (float)desktopArea) < 0.75f)
    mqWindow->show();
  else
    mqWindow->showMaximized();

//   cv::Mat im(376, 1241, CV_8UC3, cv::Scalar(255, 255, 255));
//   mqImgForm = new ImageForm(im, "Current Frame", 0, mpFrameDrawer); // mpFrameDrawer
//   mqImgForm->show();
// //  connect(this, SIGNAL(updateFrame(const cv::Mat&)), mqImgForm, SLOT(Update(const cv::Mat&)));
//   connect(this, SIGNAL(updateFrame()), mqImgForm, SLOT(Update())); // Update, repaint
//   connect(mqWindow, SIGNAL(closed()), mqImgForm, SLOT(close()));

//  connect(this, SIGNAL(updateMap(const std::vector<cv::Mat>&)), mqWindow, SLOT(drawMaps(const std::vector<cv::Mat>&)));
  connect(this, SIGNAL(updateMap()), mqWindow, SLOT(drawMaps()));
//  connect(this, SIGNAL(updateKeyFrame(const std::vector<KeyFrame*>&)), mqWindow, SLOT(drawKeyFrames(const std::vector<KeyFrame*>&)));

   cv::namedWindow("Current Frame");

//  connect(mpSystem, SIGNAL(destroyWindow()), mqWindow, SLOT(close()));

  start(); // HighestPriority
}

void Viewer::run()
{
  mbFinished = false;

  // cv::namedWindow("MY_SLAM: Current Frame");

  while(1)
  {
//    cout << "im size: " << im.size() << ", channel: " << im.channels() << endl << endl;
//    mqImgForm->Update(mImgFrame);

    // todo 2021-07-28
    std::vector<cv::Mat> vTraj;
    mpMapDrawer->getCameraTrajectories(vTraj);
//    Q_EMIT updateMap(vTraj);
    mqWindow->setTrajectories(vTraj);
    Q_EMIT updateMap();

    cv::Mat im = mpFrameDrawer->drawFrame();

//    Q_EMIT updateFrame(im);
//    mqImgForm->Update(im); // lots of memories
//    Q_EMIT updateFrame(); // lots of memories
     cv::imshow("Current Frame", im);
    QThread::usleep(mT*1e3);
//    cv::waitKey(mT);

    if(checkFinish())
    {
      cout << "checked finish" << endl;
      break;
    }
  }

  cv::destroyAllWindows();

  cout << "viewer finish" << endl;

  setFinish();

  quit();
}

void Viewer::requestFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  mbFinishRequested = true;
}

bool Viewer::isFinished()
{
  QMutexLocker locker(&mqMutexFinish);
  return mbFinished;
}

bool Viewer::checkFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  return mbFinishRequested;
}

void Viewer::setFinish()
{
  QMutexLocker locker(&mqMutexFinish);
  mbFinished = true;
}
} // namespace MY_SLAM
