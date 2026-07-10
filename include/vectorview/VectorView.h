#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#include "vectorview/Constants.h"
#include "vectorview/ForceFilter.h"

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <mutex>
#include <string>

namespace vectorview {

class VectorView : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate {
 public:
  VectorView();
  ~VectorView() override;

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

 private:
  void OnContacts(const gz::msgs::Contacts& message);
  void PublishArrow(const gz::math::Vector3d& force);
  gz::math::Vector3d ArrowPoint(const gz::math::Vector3d& begin, const gz::math::Vector3d& end,
                                double yaw_sign) const;

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher markerPub;
  gz::math::Pose3d linkWorldPose;
  std::string contactTopic;
  std::string collisionScope;
  std::string markerNamespace;
  int markerId{0};
  std::unique_ptr<ForceFilter> filter;
  std::mutex mutex;
};

}  // namespace vectorview

#endif
