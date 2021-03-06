/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMutex>

#include <vector>

#include <opencv2/core/core.hpp>

namespace MY_SLAM
{
//class QtLogo;

class GLWidget : public QGLWidget
{
  Q_OBJECT

public:
  GLWidget(QWidget *parent = 0);
  GLWidget(const std::string &strSettingPath, QWidget *parent = 0);
  virtual ~GLWidget();

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

  void setTrajectories(const std::vector<cv::Mat> & traj);

public Q_SLOTS:
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);

  void setXTranslation(int offset);
  void setZTranslation(int offset);

  void setFollowCamera(int state);

Q_SIGNALS:
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);

protected:
  void initializeGL();
  void paintGL();
  void resizeGL(int width, int height);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent *event);

private:
//  QtLogo *logo;
  int xRot;
  int yRot;
  int zRot;
  int xTran;
  int zTran;
  bool mbCheckFollowCamera;
  bool mbFollow;
  QMutex mMutexFollow;

  double dist;

  float mKeyFrameSize;
  float mKeyFrameLineWidth;
  float mGraphLineWidth;
  float mPointSize;
  float mCameraSize;
  float mCameraLineWidth;

  float mViewpointX;
  float mViewpointY;
  float mViewpointZ;
  float mViewpointF;

  float mfModelview[16];

  QPoint lastPos;
  QColor qtGreen;
  QColor qtPurple;
  QColor qtWhite;

  QMutex mMutexCamera;
  std::vector<cv::Mat> mvTraj;
  cv::Mat mCameraPose; // current camera matrix
};
} // namespace MY_SLAM

#endif
