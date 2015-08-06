#ifndef HELLOWORLD_H
#define HELLOWORLD_H

// icub includes
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
// general includes
#include <iostream>
#include <string>
#include <vector>
// specific includes
#include "Interface.h"

using namespace gazebo;

class ICubStage : public SystemPlugin
{
public:
  // constructor / destructor
  ICubStage();
  ~ICubStage();
  void Load(int _argc, char **_argv);
  void Update(ConstContactsPtr &_msg);
  void Init();

private:
  Interface *interface;
  // node that allows subscribe the sensor's messages topic and all varaibles related to transport
  std::string topic_path;
  transport::SubscriberPtr subs;
};

#endif
