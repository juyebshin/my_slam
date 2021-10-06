#ifndef IMAGEFORM_H
#define IMAGEFORM_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <iostream>

namespace MY_SLAM
{
class Viewer;
class FrameDrawer;

class ImageForm : public QWidget
{
  Q_OBJECT

private:
  QString       _q_stID;
  Viewer*       _qViewer;
  FrameDrawer*  mpFrameDrawer;

  QImage*       _q_bmpMain;
  cv::Mat       _cvMatImage;

public:
  explicit ImageForm(const std::string& strFile, Viewer *parent = 0);
  explicit ImageForm(const cv::Mat& cvMatImage, const QString q_stID, Viewer *parent = 0);
  explicit ImageForm(const cv::Mat& cvMatImage, const QString q_stID, Viewer *parent = 0, FrameDrawer *pFrameDrawer = 0);
  ~ImageForm();

  void      Update(const cv::Mat &cvMatImage);

  cv::Mat&  Image() { return _cvMatImage; }
  QString&  ID()    { return _q_stID;     }

public Q_SLOTS:
  void      Update();

protected:
  void      paintEvent(QPaintEvent* event);
  void      closeEvent(QCloseEvent *event);
  void      mousePressEvent(QMouseEvent *event);
};
} // namespace MY_SLAM

#endif // IMAGEFORM_H
