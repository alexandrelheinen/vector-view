#include "vectorview/ContactUtils.h"

namespace vectorview {

TopicNames DeriveTopicNames(const std::string& visual_name) {
  TopicNames result;
  if (visual_name.empty()) {
    return result;
  }

  std::vector<std::string> names;
  std::string name = visual_name;

  while (name.find("::") != std::string::npos) {
    names.push_back(name.substr(0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  names.push_back(name);

  if (names.empty() || names.back().empty()) {
    return result;
  }

  result.topic = "~";
  result.collision = "";
  for (size_t i = 0; i < names.size(); ++i) {
    result.topic += "/" + names[i];
    result.collision += "::" + names[i];
  }
  result.topic += "/" + names.back() + "_contact";
  result.collision += "::" + names.back() + "_collision";
  if (result.collision.size() >= 2) {
    result.collision.erase(0, 2);
  }
  result.valid = true;
  return result;
}

Vec3 AggregatePluginForces(const std::vector<ContactForce>& contacts,
                           const std::string& collision_name) {
  Vec3 force;
  if (contacts.empty()) {
    return force;
  }

  for (size_t i = 0; i < contacts.size(); ++i) {
    const ContactForce& contact = contacts[i];
    for (size_t j = 0; j < contact.wrenches.size(); ++j) {
      const WrenchForce& wrench = contact.wrenches[j];
      if (wrench.body_1_name.find(collision_name) != std::string::npos) {
        force += wrench.body_1_force;
      } else {
        force += wrench.body_2_force;
      }
    }
  }

  return force / static_cast<double>(contacts.size());
}

GuiContactResult AggregateGuiForces(const std::vector<ContactForce>& contacts,
                                    const std::string& robot_name) {
  GuiContactResult result;
  result.has_wrenches = !contacts.empty();
  if (!result.has_wrenches) {
    return result;
  }

  std::string object_name;
  for (size_t i = 0; i < contacts.size(); ++i) {
    const ContactForce& contact = contacts[i];
    for (size_t j = 0; j < contact.wrenches.size(); ++j) {
      const WrenchForce& wrench = contact.wrenches[j];
      if (wrench.body_1_name.find(robot_name) != std::string::npos) {
        object_name = wrench.body_2_name;
        result.force += wrench.body_1_force;
      } else {
        object_name = wrench.body_1_name;
        result.force += wrench.body_2_force;
      }
    }
  }

  result.object_name = object_name;
  result.force = result.force / static_cast<double>(contacts.size());
  return result;
}

}  // namespace vectorview
