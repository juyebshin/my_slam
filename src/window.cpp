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

#include <QtGui>

#include <iostream>
#include <unistd.h>

#include "glwidget.hpp"
#include "window.hpp"
#include "viewer.hpp"
#include "system.hpp"
#include "keyframe.hpp"

using namespace std;

namespace MY_SLAM
{
Window::Window()
{
  glWidget = new GLWidget;

//  xSlider = createSlider();
//  ySlider = createSlider();
//  zSlider = createSlider();

//  connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
//  connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
//  connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
//  connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
//  connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
//  connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(glWidget);
//  mainLayout->addWidget(xSlider);
//  mainLayout->addWidget(ySlider);
//  mainLayout->addWidget(zSlider);
  setLayout(mainLayout);

//  xSlider->setValue(0 * 16);
//  ySlider->setValue(0 * 16);
//  zSlider->setValue(0 * 16);
//  setWindowTitle(tr("Hello GL"));
}

Window::Window(const std::string &strSettingPath)
{
  glWidget = new GLWidget(strSettingPath);

 xSlider = createSlider();
//  ySlider = createSlider();
 zSlider = createSlider();

 connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
 connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
 connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
 connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));

 qcbFollow = new QCheckBox("Follow Camera");
 qcbFollow->setChecked(true);

 connect(qcbFollow, SIGNAL(stateChanged(int)), glWidget, SLOT(setFollowCamera(int)));

//   QHBoxLayout *mainLayout = new QHBoxLayout;
//   mainLayout->addWidget(glWidget);
//  mainLayout->addWidget(xSlider);
//  mainLayout->addWidget(zSlider);

//  QVBoxLayout *subLayout = new QVBoxLayout;
//  subLayout->addWidget(xSlider);
//  subLayout->addWidget(zSlider);

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(xSlider, 0, 0);
  gridLayout->addWidget(zSlider, 0, 1);
  gridLayout->addWidget(qcbFollow, 1, 0);
  gridLayout->addWidget(glWidget, 0, 2, 2, 2);

  setLayout(gridLayout);
}

Window::~Window()
{

}

void Window::setTrajectories(const std::vector<cv::Mat> &vTraj)
{
  mvTraj = std::vector<cv::Mat>(vTraj.begin(), vTraj.end());
}

void Window::drawMaps()
{
//  cout << "Window::drawMaps size: " << vTraj.size() << endl;
  glWidget->setTrajectories(mvTraj);
//  mvTraj = vTraj;
}

//void Window::drawMaps(const std::vector<cv::Mat> &vTraj)
//{
////  cout << "Window::drawMaps size: " << vTraj.size() << endl;
//  glWidget->setTrajectories(vTraj);
////  mvTraj = vTraj;
//}

//void Window::drawKeyFrames(const std::vector<KeyFrame*> &vKFs)
//{
//  for(int i = 0; i < vKFs.size(); i++)
//  {
//    cout << "keyframe Tcw: " << vKFs[i]->getPose() << endl;
//  }
//}

QSlider *Window::createSlider()
{
  QSlider *slider = new QSlider(Qt::Vertical);
  slider->setRange(0, 360 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}

void Window::keyPressEvent(QKeyEvent *e)
{
  if (e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
}

void Window::closeEvent(QCloseEvent *event)
{
  Q_EMIT closed();
}
} // namespace MY_SLAM
