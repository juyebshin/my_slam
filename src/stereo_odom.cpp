// stereo_odom.cpp

#include <thread>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include <QApplication>
#include <QDesktopWidget>

#include "system.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  cout << "argc: " << argc << endl;
  for(int i = 0; i < argc; i++)
    cout << argv[i] << " ";
  cout << endl;

  if(argc != 3)
  {
    cerr << "Usage: rosrun my_slam stereo_odom <path_to_settings> <path_to_sequence>" << endl;
    return 1;
  }

  MY_SLAM::System odom(argv[1], argv[2], MY_SLAM::System::STEREO);
  odom.init();

  cout << "hello" << endl;

  int ret = app.exec();

  cout << "bye" << endl;
  odom.shutdown();

  return ret;
}
