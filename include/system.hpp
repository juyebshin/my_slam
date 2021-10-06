#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <QThread>
#include <QMutex>

#include <string>
#include <thread>

#include <opencv2/core/core.hpp>

namespace MY_SLAM
{
class Tracking;
class Viewer;
class FrameDrawer;
class MapDrawer;
class Map;

class System : public QThread
{
  Q_OBJECT

public:
  enum eSensor {
    MONOCULAR = 0,
    STEREO = 1
  };
public:
  System();
  System(const std::string &strSettingsFile, const std::string &strPathToSequence, const eSensor sensor);

  virtual ~System();

  bool init();

  void run();

  void saveTrajectory(const std::string &filename);
  cv::Mat drawFrame();

  void shutdown();

Q_SIGNALS:
  void destroyWindow();

private:
  eSensor mSensor;

  std::vector<std::string> mvStrImageLeft;
  std::vector<std::string> mvStrImageRight;
  std::vector<double> mvTimestamps;

  cv::Mat mImGray;
  QMutex mqMutex;

  bool mbFinishRequested;
  bool mbFinished;
  QMutex mqMutexFinish;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map* mpMap;

  Tracking* mpTracker;
  Viewer *mpViewer;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;

  void loadStereoImages(const std::string &strPathToSequence);
  void loadMonocularImages(const std::string &strPathToSequence);

  cv::Mat trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);
  cv::Mat trackMonocular(const cv::Mat &im, const double &timestamp);

  bool requestFinish();
  bool isFinished();
  bool checkFinish();
  void setFinish();
};
} // namespace MY_SLAM

#endif // SYSTEM_HPP
