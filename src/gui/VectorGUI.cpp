#include <QtGui>
#include <gazebo.hh>
#include <iostream>
#include <string>

#include "vectorview/Interface.h"
#include "vectorview/ModelContext.h"
#include "vectorview/TopicPath.h"

using namespace gazebo;

int main(int argc, char** argv) {
  if (argc <= 1) {
    std::cout << "[TOPIC PATH] ERROR" << std::endl
              << "Please pass a contact sensor topic path or a link name to VectorGUI." << std::endl
              << "Hint: run 'gz topic -l' while gzserver is running to list topic paths." << std::endl
              << "Example full path: /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
              << std::endl
              << "Example short name: l_hand" << std::endl;
    return 1;
  }

  vectorview::ModelContext context;
  const vectorview::TopicPath path = vectorview::TopicPath::FromCliArgument(argv[1], context);
  if (!path.valid) {
    std::cout << "[TOPIC PATH] ERROR" << std::endl
              << "Could not derive a topic path from: " << argv[1] << std::endl;
    return 1;
  }

  const std::string robotName = (argc > 2) ? std::string(argv[2]) : "iCub";

  QApplication app(argc, argv);
  Interface interface(path.transport, robotName);
  interface.show();
  std::cout << " >> VectorGUI started at [" << path.transport << "] topic." << std::endl;
  return app.exec();
}
