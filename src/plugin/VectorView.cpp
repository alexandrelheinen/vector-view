#include "vectorview/VectorView.h"

#include "vectorview/ContactMessage.h"
#include "vectorview/TopicPath.h"

#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>

#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

#include <iostream>

namespace vectorview {

namespace {

void SetBoxGeometry(gz::sim::EntityComponentManager& ecm, const gz::sim::Entity visual,
                    double length, double width, double height) {
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);
  sdf::Box box;
  box.SetSize(gz::math::Vector3d(length, width, height));
  geometry.SetBoxShape(box);

  auto* geometryComp = ecm.Component<gz::sim::components::Geometry>(visual);
  if (geometryComp != nullptr) {
    geometryComp->Data() = geometry;
  } else {
    ecm.CreateComponent(visual, gz::sim::components::Geometry(geometry));
  }
  ecm.SetChanged(visual, gz::sim::components::Geometry::typeId);
}

gz::sim::Entity FindChildVisual(const gz::sim::EntityComponentManager& ecm,
                                const gz::sim::Entity parent,
                                const std::string& name) {
  const auto matches =
      ecm.ChildrenByComponents(parent, gz::sim::components::Name(name));
  for (const gz::sim::Entity child : matches) {
    if (ecm.EntityHasComponentType(child, gz::sim::components::Visual::typeId)) {
      return child;
    }
  }

  for (const gz::sim::Entity child : ecm.Descendants(parent)) {
    const auto* childName = ecm.Component<gz::sim::components::Name>(child);
    if (childName == nullptr || childName->Data() != name) {
      continue;
    }
    if (ecm.EntityHasComponentType(child, gz::sim::components::Visual::typeId)) {
      return child;
    }
  }
  return gz::sim::kNullEntity;
}

void SetVisualPose(gz::sim::EntityComponentManager& ecm, const gz::sim::Entity visual,
                   const gz::math::Pose3d& pose) {
  auto* poseComp = ecm.Component<gz::sim::components::Pose>(visual);
  if (poseComp != nullptr) {
    poseComp->Data() = pose;
  } else {
    ecm.CreateComponent(visual, gz::sim::components::Pose(pose));
  }
  ecm.SetChanged(visual, gz::sim::components::Pose::typeId);
}

gz::math::Quaterniond RotationFromXAxis(const gz::math::Vector3d& direction) {
  const gz::math::Vector3d dir = direction.Normalized();
  const gz::math::Vector3d axis = gz::math::Vector3d::UnitX.Cross(dir);
  const double sinAngle = axis.Length();
  const double cosAngle = gz::math::Vector3d::UnitX.Dot(dir);
  if (sinAngle < 1e-6) {
    return cosAngle < 0.0 ? gz::math::Quaterniond(0, 0, 1, 0)
                          : gz::math::Quaterniond::Identity;
  }
  return gz::math::Quaterniond(axis / sinAngle, std::atan2(sinAngle, cosAngle));
}

void SetSegmentVisual(gz::sim::EntityComponentManager& ecm,
                      const gz::sim::Entity visual,
                      const gz::math::Vector3d& start,
                      const gz::math::Vector3d& end,
                      const double width) {
  if (visual == gz::sim::kNullEntity) {
    return;
  }
  const gz::math::Vector3d delta = end - start;
  const double length = delta.Length();
  if (length < 1e-6) {
    return;
  }
  SetBoxGeometry(ecm, visual, length, width, width);
  SetVisualPose(
      ecm, visual,
      gz::math::Pose3d(0.5 * (start + end), RotationFromXAxis(delta)));
}

}  // namespace

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
    this->arrowShaftEntity =
        FindChildVisual(ecm, this->linkEntity, "vectorview_arrow_shaft");
    this->arrowHeadUpperEntity =
        FindChildVisual(ecm, this->linkEntity, "vectorview_arrow_head_upper");
    this->arrowHeadLowerEntity =
        FindChildVisual(ecm, this->linkEntity, "vectorview_arrow_head_lower");
    if (this->arrowShaftEntity == gz::sim::kNullEntity) {
      gzwarn << "[VectorView] Missing vectorview_arrow_shaft visual on link "
             << link_name << "\n";
    }
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
    // MarkerManager consumes /marker in the interactive GUI. Do not submit
    // marker lines to headless camera services: they use a different scene
    // frame and can render detached from the hand-linked geometry below.
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

void VectorView::PreUpdate(const gz::sim::UpdateInfo& /*info*/,
                           gz::sim::EntityComponentManager& ecm) {
  if (this->linkEntity == gz::sim::kNullEntity) {
    return;
  }

  gz::math::Vector3d force;
  bool show = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->linkWorldPose = gz::sim::worldPose(this->linkEntity, ecm);
    if (this->hasForce) {
      force = this->lastForce;
      show = true;
    }
  }

  if (show) {
    this->UpdateArrowVisuals(ecm, force);
  } else {
    this->HideArrowVisuals(ecm);
  }
}

void VectorView::PostUpdate(const gz::sim::UpdateInfo& /*info*/,
                            const gz::sim::EntityComponentManager& /*ecm*/) {
  if (this->linkEntity == gz::sim::kNullEntity) {
    return;
  }

  gz::math::Vector3d force;
  bool publish = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->hasForce) {
      force = this->lastForce;
      publish = true;
    }
  }

  if (publish) {
    this->PublishArrow(force);
  }
}

void VectorView::OnContacts(const gz::msgs::Contacts& message) {
  const std::vector<ContactForce> contacts = ContactsFromMessage(message);
  Vec3 aggregated = AggregatePluginForces(contacts, this->collisionScope);
  this->filter->Filter(&aggregated);

  gz::math::Vector3d force(aggregated.x, aggregated.y, aggregated.z);
  if (aggregated.Length() < NOISE_THRESHOLD) {
    force = gz::math::Vector3d::Zero;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->lastForce = force;
  this->hasForce = force.Length() >= NOISE_THRESHOLD;
}

gz::math::Vector3d VectorView::ArrowPoint(const gz::math::Vector3d& begin,
                                          const gz::math::Vector3d& end,
                                          double yaw_sign) const {
  const gz::math::Vector3d direction = (end - begin).Normalized();
  const gz::math::Matrix3d rotation(1, 0, 0, 0, 0.9848, yaw_sign * -0.1736, 0,
                                    yaw_sign * 0.1736, 0.9848);
  return end - ARROW_LENGTH * rotation * direction;
}

void VectorView::HideArrowVisuals(gz::sim::EntityComponentManager& ecm) const {
  const gz::math::Pose3d hidden(0, 0, -10, 0, 0, 0);
  if (this->arrowShaftEntity != gz::sim::kNullEntity) {
    SetVisualPose(ecm, this->arrowShaftEntity, hidden);
  }
  if (this->arrowHeadUpperEntity != gz::sim::kNullEntity) {
    SetVisualPose(ecm, this->arrowHeadUpperEntity, hidden);
  }
  if (this->arrowHeadLowerEntity != gz::sim::kNullEntity) {
    SetVisualPose(ecm, this->arrowHeadLowerEntity, hidden);
  }
}

void VectorView::UpdateArrowVisuals(gz::sim::EntityComponentManager& ecm,
                                    const gz::math::Vector3d& force) const {
  if (this->arrowShaftEntity == gz::sim::kNullEntity) {
    return;
  }

  const gz::math::Vector3d local_force =
      this->linkWorldPose.Rot().RotateVectorReverse(force);
  const double length = (FORCE_SCALE * local_force).Length();
  if (length < 1e-4) {
    this->HideArrowVisuals(ecm);
    return;
  }

  const gz::math::Vector3d direction = local_force.Normalized();
  const double shaftLength = std::min(std::max(length, 0.05), MAX_GEOMETRY_ARROW_LENGTH);
  constexpr double headLength = 0.08;
  constexpr double headWidth = 0.04;
  constexpr double lineWidth = 0.02;

  // Anchor the arrow tip at the hand (the point of force application).
  // The shaft extends away from the hand, so the visual stays outside the
  // contacted object instead of disappearing through its geometry.
  const gz::math::Vector3d tip = gz::math::Vector3d::Zero;
  const gz::math::Vector3d shaftStart = -shaftLength * direction;

  gz::math::Vector3d perpendicular = direction.Cross(gz::math::Vector3d::UnitZ);
  if (perpendicular.Length() < 1e-6) {
    perpendicular = direction.Cross(gz::math::Vector3d::UnitY);
  }
  perpendicular.Normalize();
  const gz::math::Vector3d headBase = -headLength * direction;
  const gz::math::Vector3d upper = headBase + headWidth * perpendicular;
  const gz::math::Vector3d lower = headBase - headWidth * perpendicular;

  SetSegmentVisual(ecm, this->arrowShaftEntity, shaftStart, tip, lineWidth);
  SetSegmentVisual(ecm, this->arrowHeadUpperEntity, upper, tip, lineWidth);
  SetSegmentVisual(ecm, this->arrowHeadLowerEntity, lower, tip, lineWidth);
}

void VectorView::PublishArrow(const gz::math::Vector3d& force) {
  const gz::math::Vector3d local_force =
      this->linkWorldPose.Rot().RotateVectorReverse(force);
  const gz::math::Vector3d end = gz::math::Vector3d::Zero;
  const gz::math::Vector3d begin = end - FORCE_SCALE * local_force;

  const auto to_world = [this](const gz::math::Vector3d& local_point) {
    return this->linkWorldPose.Pos() + this->linkWorldPose.Rot().RotateVector(local_point);
  };

  gz::msgs::Marker marker;
  marker.set_ns(this->markerNamespace);
  marker.set_id(this->markerId);
  marker.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker.set_type(gz::msgs::Marker::LINE_LIST);
  marker.set_visibility(gz::msgs::Marker::ALL);
  marker.mutable_scale()->set_x(0.04);
  marker.mutable_scale()->set_y(0.04);
  marker.mutable_scale()->set_z(0.04);
  marker.mutable_material()->mutable_ambient()->set_r(0.0f);
  marker.mutable_material()->mutable_ambient()->set_g(0.0f);
  marker.mutable_material()->mutable_ambient()->set_b(1.0f);
  marker.mutable_material()->mutable_ambient()->set_a(1.0f);
  marker.mutable_material()->mutable_diffuse()->set_r(0.0f);
  marker.mutable_material()->mutable_diffuse()->set_g(0.0f);
  marker.mutable_material()->mutable_diffuse()->set_b(1.0f);
  marker.mutable_material()->mutable_diffuse()->set_a(1.0f);
  marker.mutable_material()->mutable_emissive()->set_r(0.0f);
  marker.mutable_material()->mutable_emissive()->set_g(0.2f);
  marker.mutable_material()->mutable_emissive()->set_b(1.0f);
  marker.mutable_material()->mutable_emissive()->set_a(1.0f);

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
    this->node.Request(service, marker, 250, response, result);
  }
}

}  // namespace vectorview

GZ_ADD_PLUGIN(vectorview::VectorView, gz::sim::System, vectorview::VectorView::ISystemConfigure,
              vectorview::VectorView::ISystemPreUpdate,
              vectorview::VectorView::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(vectorview::VectorView, "vectorview::VectorView")
