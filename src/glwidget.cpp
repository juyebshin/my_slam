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

#include <pangolin/pangolin.h>

#include <QtGui>
#include <QtOpenGL>

#include <math.h>
#include <iostream>

#include <GL/glu.h>

#include <opencv2/core/core.hpp>

#include "glwidget.hpp"
//#include "qtlogo.hpp"

using namespace std;

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

#define deg2rad(deg) deg*0.017453293
#define rad2deg(rad) rad*57.295779513

namespace MY_SLAM
{
GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
  //  logo = 0;
  xRot = 0;
  yRot = 0;
  zRot = 0;
  xTran = 0;
  zTran = 0;
  dist = 1.0;

  qtGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
  qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
  qtWhite = QColor::fromRgb(255, 255, 255);
}

GLWidget::GLWidget(const std::string &strSettingPath, QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    mbCheckFollowCamera(true), mbFollow(true)
{
  xRot = 0;
  yRot = 0;
  zRot = 0;
  xTran = 0;
  zTran = 0;
  dist = 1.0;

  mCameraPose = cv::Mat::eye(4,4,CV_32F);

  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
  mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
  mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
  mPointSize = fSettings["Viewer.PointSize"];
  mCameraSize = fSettings["Viewer.CameraSize"];
  mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

  mViewpointX = fSettings["Viewer.ViewpointX"];
  mViewpointY = fSettings["Viewer.ViewpointY"];
  mViewpointZ = fSettings["Viewer.ViewpointZ"];
  mViewpointF = fSettings["Viewer.ViewpointF"];
}

GLWidget::~GLWidget()
{
}

QSize GLWidget::minimumSizeHint() const
{
  return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
  return QSize(400, 400);
}

void GLWidget::setTrajectories(const std::vector<cv::Mat> &traj)
{
  //  cout << "glWidget setTrajectories" << endl;
  QMutexLocker locker(&mMutexCamera);
  mvTraj = std::vector<cv::Mat>(traj.begin(), traj.end());
  cout << "traj size in gl: " << mvTraj.size() << endl;
  //  if(mvTraj.size())
  //  {
  //    // cv::Mat Tcw = mvTraj.back();
  //    // (mvTraj.back())->copyTo(mCameraPose);
  ////     mCameraPose = cv::Mat(4, 4, CV_32F, mvTraj.back());
  //    if(!mvTraj.back().empty())
  //      mCameraPose = mvTraj.back().clone();
  ////    memcpy(mCameraPose.data, mvTraj.back(), sizeof(float)*4*4);
  ////    cout << "setTrajectories mCameraPose: " << mCameraPose << endl;
  ////    cout << "pointer of mvTraj[i]: " << (int*)mvTraj.back() << endl;
  ////    cout << "pointer of mCameraPose: " << (int*)mCameraPose.data << endl;

  //    // cv::Mat Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
  //    // cv::Mat twc = -Rwc*mCameraPose.rowRange(0,3).col(3);

  //    // Rwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
  //    // twc.copyTo(mTwc.rowRange(0,3).col(3));
  //  }
  
  updateGL();
}

static void qNormalizeAngle(int &angle)
{
  while (angle < 0)
    angle += 360 * 16;
  while (angle > 360 * 16)
    angle -= 360 * 16;
}

void GLWidget::setXRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != xRot) {
    xRot = angle;
    Q_EMIT xRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setYRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != yRot) {
    yRot = angle;
    //   Q_EMIT yRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setZRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != zRot) {
    zRot = angle;
    Q_EMIT zRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setXTranslation(int offset)
{
  if (offset != xTran)
  {
    xTran = offset;
    updateGL();
  }
}

void GLWidget::setZTranslation(int offset)
{
  if (offset != zTran)
  {
    zTran = offset;
    updateGL();
  }
}

void GLWidget::setFollowCamera(int state)
{
  // QMutexLocker locker(&mMutexFollow);
  mbCheckFollowCamera = state;
}

void GLWidget::initializeGL()
{
  qglClearColor(qtWhite);

  //  logo = new QtLogo(this, 64);
  //  logo->setColor(qtGreen.dark());

  glEnable(GL_DEPTH_TEST);
  //    glEnable(GL_CULL_FACE);
  //    glShadeModel(GL_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //    glEnable(GL_LIGHTING);
  //    glEnable(GL_LIGHT0);
  //    glEnable(GL_MULTISAMPLE);
  //    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
  //    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void GLWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(1.0f,1.0f,1.0f,1.0f);
  glLoadIdentity();

  gluLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0);

  if(mvTraj.size())
    mCameraPose = mvTraj.back().clone();
  if(mCameraPose.empty()) return;
  float modelview[16];
  cv::Mat Rwc_ = mCameraPose.rowRange(0,3).colRange(0,3)/*.t()*/;
  cv::Mat twc_ = /*-Rwc_**/mCameraPose.rowRange(0,3).col(3);
  //  cout << "update modelview" << endl;
  modelview[0] = Rwc_.at<float>(0,0);
  modelview[1] = Rwc_.at<float>(1,0);
  modelview[2] = Rwc_.at<float>(2,0);
  modelview[3]  = 0.0;

  modelview[4] = Rwc_.at<float>(0,1);
  modelview[5] = Rwc_.at<float>(1,1);
  modelview[6] = Rwc_.at<float>(2,1);
  modelview[7]  = 0.0;

  modelview[8] = Rwc_.at<float>(0,2);
  modelview[9] = Rwc_.at<float>(1,2);
  modelview[10] =Rwc_.at<float>(2,2);
  modelview[11]  = 0.0;

  modelview[12] = twc_.at<float>(0);
  modelview[13] = twc_.at<float>(1);
  modelview[14] = twc_.at<float>(2);
  modelview[15]  = 1.0;
  // glLoadMatrixf(m);
  // cv::Mat modelViewMat(4, 4, CV_32F, m);
  cout << "current modelview mCameraPose: " << (-Rwc_.t()*twc_).t() << endl;
  //  gluLookAt(twc_.at<float>(0)+0, twc_.at<float>(1)-100, twc_.at<float>(2)-0.1, twc_.at<float>(0), twc_.at<float>(1), twc_.at<float>(2), 0.0, -1.0, 0.0);
  if(mbCheckFollowCamera && mbFollow)
  {
    // gluLookAt(mTwc.at<float>(0)+0, mTwc.at<float>(1)-100, mTwc.at<float>(2)-0.1, mTwc.at<float>(0), mTwc.at<float>(1), mTwc.at<float>(2), 0.0, -1.0, 0.0);
    //    gluLookAt(0, -100, -0.1, 0, 0, 0, 0.0, -1.0, 0.0);
    // cv::Mat Tvc =
    glMultMatrixf(modelview);
  }
  else if(mbCheckFollowCamera && !mbFollow)
  {
    // gluLookAt(0, -100, -0.1, mTwc.at<float>(0), mTwc.at<float>(1), mTwc.at<float>(2), 0.0, -1.0, 0.0);
    //    gluLookAt(0, -100, -0.1, 0, 0, 0, 0.0, -1.0, 0.0);
    glMultMatrixf(modelview);
    xRot = 0;
    yRot = 0;
    zRot = 0;
    xTran = 0;
    zTran = 0;
    dist = 1.0;
    mbFollow = true;
  }
  else if(!mbCheckFollowCamera && mbFollow)
  {
    mbFollow = false;
    xRot = 0;
    yRot = 0;
    zRot = 0;
    xTran = 0;
    zTran = 0;
    //    dist = 1.0;
  }
  else
  {
    glTranslatef(xTran*0.1, 0.0, zTran*0.1);
    //    glScalef(dist, dist, dist);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    //  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
  }
  glScalef(dist, dist, dist);

  //  cout << " rx: " << xRot / 16.
  //       << " ry: " << yRot / 16.
  //       << " rz: " << zRot / 16. << endl;
  //    logo->draw();

  //    glMultMatrixf()

  const float &w = mCameraSize;
  const float h = w*0.75;
  const float z = w*0.6;

  // QMutexLocker locker(&mMutexCamera);
  cout << "updateGL mvTraj.size(): " << mvTraj.size() << endl;
  for(size_t i = 0, iend = mvTraj.size(); i < iend; i++)
  {
    //    cv::Mat Tcw(4, 4, CV_32F);
    cv::Mat Tcw = mvTraj[i].clone();
    //    memcpy(Tcw.data, mvTraj[i], sizeof(float)*4*4);
    //    cout << "P " << (int)i << ": " << Tcw << endl;
    //    cout << "P rowrange" << (int)i << ": " << Tcw.rowRange(0,3) << endl;
    //    cout << "pointer of mvTraj[i]: " << (int*)mvTraj[i] << endl;
    //    cout << "pointer of Tcw: " << (int*)Tcw.data << endl;
    
    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);

    Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    twc = -Rwc*Tcw.rowRange(0,3).col(3);
    
    float m[16];
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

    glPushMatrix();
    glMultMatrixf(m);

    glLineWidth(mCameraLineWidth);
    // glClearColor(1.0f,1.0f,1.0f,1.0f);
    glBegin(GL_LINES);
    // Camera shape
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    // x: red
    glColor3f(1., 0., 0.);
    glVertex3f(w, 0., 0.);
    glVertex3f(0., 0., 0.);
    // y: green
    glColor3f(0., 1., 0.);
    glVertex3f(0., w, 0.);
    glVertex3f(0., 0., 0.);
    // z: blue
    glColor3f(0., 0., 1.);
    glVertex3f(0., 0., w);
    glVertex3f(0., 0., 0.);

    glEnd();

    glPopMatrix();
  }
  cout << "draw done" << endl;

  // glLineWidth(3.);
  // glClearColor(1.0f,1.0f,1.0f,1.0f);
  // glBegin(GL_LINES);
  // // x: red
  // glColor3f(1., 0., 0.);
  // glVertex3f(1., 0., 0.);
  // glVertex3f(0., 0., 0.);
  // // y: green
  // glColor3f(0., 1., 0.);
  // glVertex3f(0., 1., 0.);
  // glVertex3f(0., 0., 0.);
  // // z: blue
  // glColor3f(0., 0., 1.);
  // glVertex3f(0., 0., 1.);
  // glVertex3f(0., 0., 0.);

  // glEnd();

  // glPushMatrix();
  // const float m[16] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 5.0, 5.0, 1.0};
  // glMultMatrixf(m);

  // glLineWidth(3.);
  // glClearColor(1.0f,1.0f,1.0f,1.0f);
  // glBegin(GL_LINES);
  // // x: red
  // glColor3f(1., 0., 0.);
  // glVertex3f(1., 0., 0.);
  // glVertex3f(0., 0., 0.);
  // // y: green
  // glColor3f(0., 1., 0.);
  // glVertex3f(0., 1., 0.);
  // glVertex3f(0., 0., 0.);
  // // z: blue
  // glColor3f(0., 0., 1.);
  // glVertex3f(0., 0., 1.);
  // glVertex3f(0., 0., 0.);

  // glEnd();

  // glPopMatrix();
}

void GLWidget::resizeGL(int width, int height)
{
  int side = qMax(width, height);
  glViewport((width - side) / 2, (height - side) / 2, side, side);
  //  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
#ifdef QT_OPENGL_ES_1
  //    glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 30.0);
  glFrustumf(-0.5, +0.5, -0.5, +0.5, 4.0, 200.0); // 0.5
#else
  //    glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 30.0);
  glFrustum(-0.5, +0.5, -0.5, +0.5, 4.0, 200.0);
#endif
  glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->x() - lastPos.x();
  int dy = event->y() - lastPos.y();

  if(!mbFollow)
  {
    if (event->buttons() & Qt::LeftButton) {
      setXTranslation(xTran + dx);
      setZTranslation(zTran - dy);
    } else if (event->buttons() & Qt::RightButton) {
      setXRotation(xRot + 8 * dy);
      setZRotation(zRot + 8 * dx);
    }
  }
  lastPos = event->pos();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
  int deg = event->delta() / 8;
  int step = deg / 15;

  dist = dist + 0.1*step;
  //  dist = dist - step;
  if(dist < 0.01)
    dist = 0.01;
  else if(dist > 20)
    dist = 20;

  cout << "scroll: " << dist << endl;

  updateGL();
  //  resizeGL(this->width(), this->height());
}
} // namespace MY_SLAM
