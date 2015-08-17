#include <QtGui>
#include <string>
#include <gazebo.hh>
#include "Interface.h"

using namespace gazebo;
int main(int _argc, char** _argv)
{
  std::string path = "~/iCub_fixed/iCub/r_hand/r_hand_contact";
  QApplication app(_argc, _argv);
  Interface interface(path);
  interface.show();
  return app.exec();
}
