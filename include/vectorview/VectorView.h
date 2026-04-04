#ifndef VECTORVIEW_H
#define VECTORVIEW_H

/** @brief Scale factor: scene units per Newton (m·N⁻¹). */
#define FORCE_SCALE 8E-2
/** @brief Half-arrowhead length in scene units. */
#define ARROW_LENGTH .05

// Gazebo includes
#include <gazebo.hh>
#include <rendering/Visual.hh>
#include <rendering/rendering.hh>
#include <msgs/msgs.hh>
#include <common/common.hh>
// ignition math (Gazebo 9+)
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
// standard includes
#include <iostream>
#include <memory>
#include <string>
#include <vector>
// local includes
#include "common/Constants.h"
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include "DspFilters/ForceFilter.h"
#include "vectorview/NameUtils.h"

namespace gazebo
{
  class VectorView : public VisualPlugin
  {
  public:
    /** @brief Default constructor. */
    VectorView();
    /** @brief Destructor. Releases owned resources. */
    ~VectorView();

    /**
     * @brief Called once when the plugin is loaded; stores the parent visual.
     * @param _parent Parent visual pointer.
     * @param _sdf    SDF element (unused).
     */
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief Initialises visual lines and subscribes to the contact sensor topic.
     *
     * Called after Load(); sets up the DynamicLines arrowhead, derives the
     * topic name from the visual name and subscribes to it.
     */
    void Init();

    /**
     * @brief Callback invoked for every contact sensor message.
     * @param _msg Incoming contacts message.
     */
    void VectorViewUpdate(ConstContactsPtr& _msg);

  private:
    /**
     * @brief Derives topicName and collisionName from the parent visual name.
     *
     * Delegates to VectorView::ParseVisualName().  Logs an error and returns
     * early if the visual name cannot be parsed.
     */
    void FindName();

    /**
     * @brief Updates the DynamicLines arrowhead to represent @p force.
     * @param force Force vector in world coordinates (N).
     */
    void UpdateVector(ignition::math::Vector3d force);

    // DynamicLines pointer is a non-owning (borrowing) pointer — the
    // DynamicLines object is owned by the parent Visual.
    rendering::DynamicLines* forceVector;

    rendering::VisualPtr visual;
    transport::SubscriberPtr subs;
    transport::NodePtr node;

    std::string collisionName;
    std::string topicName;

    std::unique_ptr<Dsp::ForceFilter> filter;
  };
}

#endif
