#ifndef _LINKMANAGER_
#define _LINKMANAGER_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  /// \brief An example plugin for a Link sensor.
  class LinkManager : public ModelPlugin
  {
  public:
    LinkManager();
    ~LinkManager();

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();

  private:
  /// \brief Pointer to the Link sensor
    physics::LinkPtr r_hand;
    physics::LinkPtr l_hand;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::ofstream *output_history;
  };
}
#endif
