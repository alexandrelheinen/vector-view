#include <QApplication>
#include <string>
#include <gazebo.hh>
#include <iostream>
#include "vectorGUI/Interface.h"

int main(int _argc, char** _argv)
{
  if (_argc > 1)
  {
    std::string path(_argv[1]);
    if (path.find("/") == std::string::npos)
      path = "/gazebo/default/iCub_fixed/iCub/" + path + "/" + path + "_contact";

    std::string robotName = (_argc > 2) ? std::string(_argv[2]) : "iCub";

    QApplication app(_argc, _argv);
    Interface interface(path, robotName);
    interface.show();
    std::cout << " >> VectorGUI started at [" << path << "] topic." << std::endl;
    return app.exec();
  }
  else
  {
    std::cout << "[TOPIC PATH] ERROR"                                                                                         << std::endl
              << "Please pass contact sensor 'topic path' or the link name as input of VectorGUI."                            << std::endl
              << "Hint: Type 'gz topic -l' while Gazebo server (or gzserver) is running to display topic paths list."         << std::endl
              << "      Your interest topic should be something like /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact."  << std::endl
              << "      Optionally pass the robot model name as a second argument (default: iCub)."                           << std::endl;
  }
}
