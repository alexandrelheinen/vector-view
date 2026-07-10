#include "vectorview/VectorView.h"

#include "vectorview/ContactUtils.h"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView() {}

VectorView::~VectorView() {}

void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {
  (void)_sdf;
  this->visual = _parent;
  filter.reset(new vectorview::ForceFilter());
}

void VectorView::Init() {
  this->FindName();

  this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for (int k = 0; k < 6; ++k) {
    this->forceVector->AddPoint(math::Vector3::Zero);
  }

  this->forceVector->Update();

  this->subs.reset();
  node.reset(new gazebo::transport::Node());
  node->Init();
  this->subs = node->Subscribe(topicName, &VectorView::VectorViewUpdate, this);
  std::cout << std::endl
            << "-- VectorView plugin initialized" << std::endl
            << "   topic path : " << topicName << std::endl
            << "   collision  : " << collisionName << std::endl
            << std::endl;
}

void VectorView::UpdateVector(math::Vector3 force) {
  math::Vector3 begin = math::Vector3::Zero;
  math::Vector3 end =
      begin + FORCE_SCALE * (visual->GetWorldPose().rot.RotateVectorReverse(force));
  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end);
  this->forceVector->SetPoint(
      3, end - ARROW_LENGTH * math::Matrix3(1, 0, 0, 0, 0.9848, -0.1736, 0, 0.1736, 0.9848) *
               (end - begin).Normalize());
  this->forceVector->SetPoint(4, end);
  this->forceVector->SetPoint(
      5, end - ARROW_LENGTH * math::Matrix3(1, 0, 0, 0, 0.9848, 0.1736, 0, -0.1736, 0.9848) *
               (end - begin).Normalize());
}

void VectorView::FindName() {
  vectorview::TopicNames names = vectorview::DeriveTopicNames(this->visual->GetName());
  if (!names.valid) {
    gzerr << "[VectorView] Could not parse visual name.\n";
    return;
  }
  topicName = names.topic;
  collisionName = names.collision;
}

void VectorView::VectorViewUpdate(ConstContactsPtr &message) {
  std::vector<vectorview::ContactForce> contacts;
  contacts.reserve(message->contact_size());

  for (int n = 0; n < message->contact_size(); ++n) {
    vectorview::ContactForce contact;
    for (int m = 0; m < message->contact(n).wrench_size(); ++m) {
      vectorview::WrenchForce wrench;
      const msgs::Wrench &body_1_wrench = message->contact(n).wrench(m).body_1_wrench();
      const msgs::Wrench &body_2_wrench = message->contact(n).wrench(m).body_2_wrench();
      wrench.body_1_name = message->contact(n).wrench(m).body_1_name();
      wrench.body_2_name = message->contact(n).wrench(m).body_2_name();
      wrench.body_1_force = vectorview::Vec3(body_1_wrench.force().x(), body_1_wrench.force().y(),
                                             body_1_wrench.force().z());
      wrench.body_2_force = vectorview::Vec3(body_2_wrench.force().x(), body_2_wrench.force().y(),
                                             body_2_wrench.force().z());
      contact.wrenches.push_back(wrench);
    }
    contacts.push_back(contact);
  }

  vectorview::Vec3 aggregated =
      vectorview::AggregatePluginForces(contacts, this->collisionName);

  filter->Filter(&aggregated);

  if (aggregated.Length() < NOISE_THRESHOLD) {
    aggregated = vectorview::Vec3();
  }

  math::Vector3 force(aggregated.x, aggregated.y, aggregated.z);
  this->UpdateVector(force);
}
