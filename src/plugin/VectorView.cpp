#include "vectorview/VectorView.h"

#include "vectorview/ContactMessage.h"
#include "vectorview/TopicPath.h"

#include <gz/math/Matrix3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <iostream>

namespace vectorview {

VectorView::VectorView() = default;

VectorView::~VectorView() = default;

void VectorView::Configure(const gz::sim::Entity& entity,
                           const std::shared_ptr<const sdf::Element>& sdf,
                           gz::sim::EntityComponentManager& ecm,
                           gz::sim::EventManager& /*eventMgr*/) {
  this->modelEntity = entity;
  this->filter = std::make_unique<ForceFilter>();

  std::string link_name;
  if (sdf->HasElement("link_name")) {
    link_name = sdf->Get<std::string>("link_name");
  }

  ModelContext context;
  if (sdf->HasElement("contact_topic")) {
    this->contactTopic = sdf->Get<std::string>("contact_topic");
  } else if (!link_name.empty()) {
    const TopicPath path = TopicPath::FromLinkName(link_name, context);
    this->contactTopic = path.transport;
  }

  if (sdf->HasElement("collision_scope")) {
    this->collisionScope = sdf->Get<std::string>("collision_scope");
  } else if (!link_name.empty()) {
    const TopicPath path = TopicPath::FromLinkName(link_name, context);
    this->collisionScope = path.collision_scope;
  }

  if (!link_name.empty()) {
    gz::sim::Model model(entity);
    this->linkEntity = model.LinkByName(ecm, link_name);
  }

  if (this->contactTopic.empty()) {
    gzerr << "[VectorView] Missing contact_topic or link_name in plugin SDF.\n";
    return;
  }

  this->markerNamespace = "vectorview/" + link_name;
  this->markerId = static_cast<int>(std::hash<std::string>{}(this->markerNamespace) % 100000);

  if (sdf->HasElement("marker_service")) {
    this->markerServices.push_back(sdf->Get<std::string>("marker_service"));
  } else {
    // Headless camera recording uses the sensors render scene; interactive GUI
    // mode uses the main scene. Try both so arrows appear in either workflow.
    this->markerServices.push_back("/sensors/marker");
    this->markerServices.push_back("/marker");
  }

  this->node.Subscribe(this->contactTopic,
                       &VectorView::OnContacts, this);

  std::cout << std::endl
            << "-- Vector View system initialized" << std::endl
            << "   contact topic : " << this->contactTopic << std::endl
            << "   collision     : " << this->collisionScope << std::endl
            << std::endl;
}

void VectorView::PostUpdate(const gz::sim::UpdateInfo& /*info*/,
                            const gz::sim::EntityComponentManager& ecm) {
  if (this->linkEntity == gz::sim::kNullEntity) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->linkWorldPose = gz::sim::worldPose(this->linkEntity, ecm);
}

void VectorView::OnContacts(const gz::msgs::Contacts& message) {
  const std::vector<ContactForce> contacts = ContactsFromMessage(message);
  Vec3 aggregated = AggregatePluginForces(contacts, this->collisionScope);
  this->filter->Filter(&aggregated);

  if (aggregated.Length() < NOISE_THRESHOLD) {
    aggregated = Vec3();
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->PublishArrow(gz::math::Vector3d(aggregated.x, aggregated.y, aggregated.z));
}

gz::math::Vector3d VectorView::ArrowPoint(const gz::math::Vector3d& begin,
                                          const gz::math::Vector3d& end,
                                          double yaw_sign) const {
  const gz::math::Vector3d direction = (end - begin).Normalized();
  const gz::math::Matrix3d rotation(1, 0, 0, 0, 0.9848, yaw_sign * -0.1736, 0,
                                    yaw_sign * 0.1736, 0.9848);
  return end - ARROW_LENGTH * rotation * direction;
}

void VectorView::PublishArrow(const gz::math::Vector3d& force) {
  const gz::math::Vector3d begin = gz::math::Vector3d::Zero;
  const gz::math::Vector3d local_force =
      this->linkWorldPose.Rot().RotateVectorReverse(force);
  gz::math::Vector3d shaft = FORCE_SCALE * local_force;
  const double shaft_length = shaft.Length();
  if (shaft_length > MAX_ARROW_LENGTH && shaft_length > 0.0) {
    shaft = shaft / shaft_length * MAX_ARROW_LENGTH;
  }
  const gz::math::Vector3d end = begin + shaft;

  const auto to_world = [this](const gz::math::Vector3d& local_point) {
    return this->linkWorldPose.Pos() + this->linkWorldPose.Rot().RotateVector(local_point);
  };

  gz::msgs::Marker marker;
  marker.set_ns(this->markerNamespace);
  marker.set_id(this->markerId);
  marker.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker.set_type(gz::msgs::Marker::LINE_LIST);
  marker.mutable_material()->mutable_ambient()->set_r(0.0f);
  marker.mutable_material()->mutable_ambient()->set_g(0.0f);
  marker.mutable_material()->mutable_ambient()->set_b(1.0f);
  marker.mutable_material()->mutable_ambient()->set_a(1.0f);
  marker.mutable_material()->mutable_diffuse()->set_r(0.0f);
  marker.mutable_material()->mutable_diffuse()->set_g(0.0f);
  marker.mutable_material()->mutable_diffuse()->set_b(1.0f);
  marker.mutable_material()->mutable_diffuse()->set_a(1.0f);

  auto set_point = [&marker](const gz::math::Vector3d& point) {
    gz::msgs::Vector3d* msg_point = marker.add_point();
    msg_point->set_x(point.X());
    msg_point->set_y(point.Y());
    msg_point->set_z(point.Z());
  };

  set_point(to_world(begin));
  set_point(to_world(end));
  set_point(to_world(end));
  set_point(to_world(ArrowPoint(begin, end, 1.0)));
  set_point(to_world(end));
  set_point(to_world(ArrowPoint(begin, end, -1.0)));

  gz::msgs::Empty response;
  bool result = false;
  for (const std::string& service : this->markerServices) {
    if (this->node.Request(service, marker, 100, response, result) && result) {
      return;
    }
  }
}

}  // namespace vectorview

GZ_ADD_PLUGIN(vectorview::VectorView, gz::sim::System, vectorview::VectorView::ISystemConfigure,
              vectorview::VectorView::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(vectorview::VectorView, "vectorview::VectorView")
