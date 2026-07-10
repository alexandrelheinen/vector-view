#include <QApplication>

#include <iostream>
#include <string>

#include "vectorview/Interface.h"
#include "vectorview/ModelContext.h"
#include "vectorview/TopicPath.h"

int main(int argc, char** argv) {
  if (argc <= 1) {
    std::cout << "[TOPIC PATH] ERROR" << std::endl
              << "Please pass a contact sensor topic path or a link name to VectorGUI." << std::endl
              << "Hint: run 'gz topic -l' while gz sim is running to list topic paths." << std::endl
              << "Example full path: /vectorview/iCub_fixed/r_hand" << std::endl
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

  const std::string robotName = (argc > 2) ? std::string(argv[2]) : context.robot_model;
  const std::string worldName = (argc > 3) ? std::string(argv[3]) : context.world_name;

  QApplication app(argc, argv);
  Interface interface(path.transport, robotName, worldName);
  interface.show();
  std::cout << " >> VectorGUI started at [" << path.transport << "] topic." << std::endl;
  return app.exec();
}
