// mono_odom.cpp
#include <iostream>

// Qt
#include <QApplication>
#include <QDesktopWidget>

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    cerr << "Usage: rosrun my_slam mono_odom <path_to_settings> <path_to_sequence>" << endl;
    return 1;
  }

  int nSeq = 0;
  char chSeq[2];
  for(int i = 0; i < strlen(argv[2]); i++)
  {
    if(argv[2][i] > 47 && argv[2][i] < 58) nSeq = nSeq*10 + argv[2][i]-48;
  }
  sprintf(chSeq, "%02d", nSeq);

  MY_SLAM::System odom(argv[1], argv[2], MY_SLAM::System::MONOCULAR);
  odom.init();

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
//  app.connect(&odom, SIGNAL(destroyWindow()), &app, SLOT(quit()));
  int ret = app.exec();
  // default directory is /home/<user>/catkin_ws/
  odom.saveTrajectory("src/my_slam/result/" + string(chSeq) + "/CameraTrajectory.txt");
  odom.shutdown();

  return ret;
}
