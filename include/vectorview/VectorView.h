#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#define FORCE_SCALE 3E-2 // scale between the forces intensities and the vectors length (unity N^-1)
#define NOISE_THRESHOLD 1E-3
#define ARROW_LENGTH .05

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
// filter includes
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"

namespace gazebo
{
  typedef gazebo::msgs::Contacts* ContactsPtr;

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
    ContactsPtr contacts;  // current contacts
    rendering::DynamicLines* forceVector; // the vector representation line, a vector is used because the same sensor can have many contacts
    rendering::VisualPtr visual;
    std::ofstream *output_history;
    transport::SubscriberPtr subs;
    transport::NodePtr node;

    std::string collisionName;
    std::string fileName;
    std::string topicName;
    // filters
    double fc;
    Dsp::Filter* filter;
  };
}

#endif
