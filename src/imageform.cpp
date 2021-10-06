#include "imageform.hpp"
#include "viewer.hpp"
#include "framedrawer.hpp"

using namespace std;
using namespace cv;

namespace MY_SLAM
{
ImageForm::ImageForm(const std::string& strFile, Viewer *parent)
{
  // this->setEnabled(true);

  QString q_stFile = QString::fromStdString(strFile);
  setWindowTitle(q_stFile);
  _q_stID     = q_stFile;

  _qViewer = parent;

  //  _cvMatImage = imread(strFile, CV_LOAD_IMAGE_UNCHANGED);

  //  if(_cvMatImage.channels() == 1)
  //  {
  //    cout << "gray" << endl;
  //    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, QImage::Format_Indexed8);
  //  }
  //  else if(_cvMatImage.channels() == 3)
  //  {
  //    cout << "rgb" << endl;
  //    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, QImage::Format_RGB888);
  //  }
  _q_bmpMain   = new QImage;
  _q_bmpMain->load(q_stFile);

  if(_q_bmpMain->format() == QImage::Format_Indexed8)
  {
    _cvMatImage = Mat(_q_bmpMain->height(), _q_bmpMain->width(), CV_8UC1, _q_bmpMain->bits(), _q_bmpMain->bytesPerLine());
    for(int i = 0; i < 256; i++)
    {
      QRgb value = qRgb(i, i, i);
      _q_bmpMain->setColor(i, value);
    }
  }
  else if(_q_bmpMain->format() == QImage::Format_RGB888)
  {
    _cvMatImage = Mat(_q_bmpMain->height(), _q_bmpMain->width(), CV_8UC3, _q_bmpMain->bits(), _q_bmpMain->bytesPerLine());
  }

  resize(_q_bmpMain->width(), _q_bmpMain->height());

//  static int nY = -_q_bmpMain->height();
//  QRect   q_rcGeom = _qWindow->geometry();

//  setGeometry(q_rcGeom.left(), q_rcGeom.top() + (nY+=_q_bmpMain->height()), _q_bmpMain->width(), _q_bmpMain->height());
}

ImageForm::ImageForm(const Mat& cvMatImage, const QString q_stID, Viewer *parent)
{
  // this->setEnabled(true);

  setWindowTitle(q_stID);
  _q_stID     = q_stID;

  _qViewer = parent;

  _cvMatImage = cvMatImage;
  if(_cvMatImage.channels() == 1)
  {
    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_Indexed8);
    for(int i = 0; i < 256; i++)
    {
      QRgb value = qRgb(i, i, i);
      _q_bmpMain->setColor(i, value);
    }
  }
  else if(_cvMatImage.channels() == 3)
  {
    QImage *temp = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_RGB888);
    _q_bmpMain = new QImage(temp->rgbSwapped());
  }

  resize(_q_bmpMain->width(), _q_bmpMain->height());

//  static int nY = -_q_bmpMain->height();
//  QRect   q_rcGeom = _qWindow->geometry();

//  setGeometry(q_rcGeom.left(), q_rcGeom.top() + (nY+=_q_bmpMain->height()), _q_bmpMain->width(), _q_bmpMain->height());
}

ImageForm::ImageForm(const cv::Mat& cvMatImage, const QString q_stID, Viewer *parent, FrameDrawer *pFrameDrawer)
  : mpFrameDrawer(pFrameDrawer)
{
  // this->setEnabled(true);

  setWindowTitle(q_stID);
  _q_stID     = q_stID;

  _qViewer = parent;

  _cvMatImage = cvMatImage;
  if(_cvMatImage.channels() == 1)
  {
    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_Indexed8);
    for(int i = 0; i < 256; i++)
    {
      QRgb value = qRgb(i, i, i);
      _q_bmpMain->setColor(i, value);
    }
  }
  else if(_cvMatImage.channels() == 3)
  {
    QImage *temp = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_RGB888);
    _q_bmpMain = new QImage(temp->rgbSwapped());
  }

  resize(_q_bmpMain->width(), _q_bmpMain->height());

//  static int nY = -_q_bmpMain->height();
//  QRect   q_rcGeom = _qWindow->geometry();

//  setGeometry(q_rcGeom.left(), q_rcGeom.top() + (nY+=_q_bmpMain->height()), _q_bmpMain->width(), _q_bmpMain->height());
}

ImageForm::~ImageForm()
{
//  delete ui;
}

void ImageForm::Update(const cv::Mat &cvMatImage)
{
//  if(_cvMatImage.rows != cvMatImage.rows || _cvMatImage.cols != cvMatImage.cols)
//  {
//    cout << "Image update error" << endl;
//    return ;
//  }

  // cout << "update" << endl;
//  if(!mpFrameDrawer) return;
  _cvMatImage = cvMatImage.clone();
  if(_cvMatImage.channels() == 1)
  {
    // cout << "update gray" << endl;
    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_Indexed8);
    for(int i = 0; i < 256; i++)
    {
      QRgb value = qRgb(i, i, i);
      _q_bmpMain->setColor(i, value);
    }
  }
  else if(_cvMatImage.channels() == 3)
  {
    // cout << "update rgb" << endl;
    QImage *temp = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_RGB888);
    _q_bmpMain = new QImage(temp->rgbSwapped());
  }

  resize(_q_bmpMain->width(), _q_bmpMain->height());

  // cout << "repaint" << endl;

//  repaint();
//  show();
}

void ImageForm::Update()
{
//  if(_cvMatImage.rows != cvMatImage.rows || _cvMatImage.cols != cvMatImage.cols)
//  {
//    cout << "Image update error" << endl;
//    return ;
//  }

  // cout << "update" << endl;
  if(!mpFrameDrawer) return;
  _cvMatImage = mpFrameDrawer->drawFrame();
  if(_cvMatImage.channels() == 1)
  {
    // cout << "update gray" << endl;
    _q_bmpMain = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_Indexed8);
    for(int i = 0; i < 256; i++)
    {
      QRgb value = qRgb(i, i, i);
      _q_bmpMain->setColor(i, value);
    }
  }
  else if(_cvMatImage.channels() == 3)
  {
    // cout << "update rgb" << endl;
    QImage *temp = new QImage((const unsigned char*)_cvMatImage.data, _cvMatImage.cols, _cvMatImage.rows, static_cast<int>(_cvMatImage.step), QImage::Format_RGB888);
    _q_bmpMain = new QImage(temp->rgbSwapped());
  }

  resize(_q_bmpMain->width(), _q_bmpMain->height());

  // cout << "repaint" << endl;

  repaint();
//  show();
}

void ImageForm::paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event) ;

  QPainter  painter;
  painter.begin(this);
  painter.drawImage(0, 0, *_q_bmpMain);

  painter.end();
}

void ImageForm::closeEvent(QCloseEvent *event)
{
  Q_UNUSED(event) ;
//  _qWindow->closeImageForm(this);
  cout << "close event" << endl;
//  _qViewer->closeImageForm();
}

void ImageForm::mousePressEvent(QMouseEvent *event)
{
  int x = event->x();
  int y = event->y();

  //    _q_MainFrame->OnMousePos(x, y, this);
}
} // namespace MY_SLAM
