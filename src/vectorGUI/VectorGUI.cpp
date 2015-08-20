#include <QtGui>
#include <string>
#include <gazebo.hh>
#include <iostream>
#include "vectorGUI/Interface.h"

using namespace gazebo;
int main(int _argc, char** _argv)
{
  if(_argc > 1)
  {
    std::string path(_argv[1]);
    if(path.find("/") == std::string::npos)
      path = "/gazebo/default/iCub_fixed/iCub/" + path + "/" + path + "_contact";

    QApplication app(_argc, _argv);
    Interface interface(path);
    interface.show();
    std::cout << " >> VectorGUI started at [" << path << "] topic." << std::endl;
    return app.exec();
  } else
  {
    std::cout << "[TOPIC PATH] ERROR"                                                                                         << std::endl
              << "Please pass contact sensor 'topic path' or the link name as input of VectorGUI."                            << std::endl
              << "Hint: Type 'gz topic -l' while Gazebo server (or gzserver) is running to display topic paths list."         << std::endl
              << "      Your insterest topic should be something like /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact." << std::endl;
  }
}
