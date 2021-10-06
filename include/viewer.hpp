#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <QThread>
#include <QMutex>

#include <vector>

#include <opencv2/core/core.hpp>

class QApplication;

namespace MY_SLAM
{
class Window;
class ImageForm;
class System;
class FrameDrawer;
class MapDrawer;
class KeyFrame;

class Viewer : public QThread
{
  Q_OBJECT

public:
  Viewer();
  Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, const std::string &strSettingPath);
  virtual ~Viewer();

  bool init();
  void run();

  void requestFinish();

  bool isFinished();

  inline void closeImageForm() {
    if(mqImgForm) {
      delete mqImgForm;
      mqImgForm = 0;
    }
  }

Q_SIGNALS:
//  void updateFrame(const cv::Mat&);
//  void updateMap(const std::vector<cv::Mat>&);
  void updateKeyFrame(const std::vector<KeyFrame*>&);

  void updateFrame();
  void updateMap();
  void updateKeyFrame();

private:
  bool checkFinish();
  void setFinish();

  System *mpSystem;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;
  Window *mqWindow;
  ImageForm *mqImgForm;

  std::string mStrSettingPath;

  double mT;

  bool mbFinishRequested;
  bool mbFinished;
  QMutex mqMutexFinish;
};
} // namespace MY_SLAM

#endif // VIEWER_HPP
