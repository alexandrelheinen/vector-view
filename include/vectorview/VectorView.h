#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#include "vectorview/Constants.h"

// Gazebo includes
#include <gazebo.hh>
#include <rendering/Visual.hh>
#include <rendering/rendering.hh>
#include <msgs/msgs.hh>
#include <common/common.hh>
// general includes
#include <iostream>
#include <string>
#include <vector>
// #include <boost/shared_ptr.hpp>
// filter includes
#include "vectorview/ForceFilter.h"

namespace gazebo
{
  //typedef boost::shared_ptr<rendering::DynamicLines> LinePtr;
  typedef rendering::DynamicLines* LinePtr;

  class VectorView : public VisualPlugin
  {
  public:
    // CONSTRUCTOR AND DESTRUCTOR
    VectorView();
    ~VectorView();
    // FUNCTIONS
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf); // executed once the plugin is loaded
    void VectorViewUpdate(ConstContactsPtr &_msg);                 // executed everytime a message is published by the sensor: updates the vector visual
    void Init();

  private:
    // FUNCTIONS
    void FindName();     // find the topic, output history and collision names based on the visual
    void UpdateVector(math::Vector3 force);  // update visual from the vector
    // VARIABLES
    LinePtr forceVector; // the animated line representing the force
    rendering::VisualPtr visual;
    transport::SubscriberPtr subs;
    transport::NodePtr node;

    std::string collisionName;
    std::string topicName;

    vectorview::ForceFilter* filter;
  };
}

#endif
