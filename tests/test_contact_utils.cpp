#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "vectorview/ContactUtils.h"

TEST_CASE("DeriveTopicNames parses nested visual names", "[contact_utils]") {
  const vectorview::TopicNames names =
      vectorview::DeriveTopicNames("iCub::l_hand::l_hand_visual");

  REQUIRE(names.valid);
  REQUIRE(names.topic == "~/iCub/l_hand/l_hand_visual/l_hand_visual_contact");
  REQUIRE(names.collision == "iCub::l_hand::l_hand_visual::l_hand_visual_collision");
}

TEST_CASE("DeriveTopicNames handles single-segment names", "[contact_utils]") {
  const vectorview::TopicNames names = vectorview::DeriveTopicNames("l_hand_visual");

  REQUIRE(names.valid);
  REQUIRE(names.topic == "~/l_hand_visual/l_hand_visual_contact");
  REQUIRE(names.collision == "l_hand_visual::l_hand_visual_collision");
}

TEST_CASE("DeriveTopicNames rejects empty names", "[contact_utils]") {
  const vectorview::TopicNames names = vectorview::DeriveTopicNames("");
  REQUIRE_FALSE(names.valid);
}

TEST_CASE("AggregatePluginForces averages contact forces", "[contact_utils]") {
  vectorview::ContactForce contact;
  vectorview::WrenchForce wrench;
  wrench.body_1_name = "table::collision";
  wrench.body_2_name = "iCub::l_hand::l_hand_collision";
  wrench.body_1_force = vectorview::Vec3(0.0, 0.0, 0.0);
  wrench.body_2_force = vectorview::Vec3(3.0, 0.0, 0.0);
  contact.wrenches.push_back(wrench);

  std::vector<vectorview::ContactForce> contacts(1, contact);
  const vectorview::Vec3 force =
      vectorview::AggregatePluginForces(contacts, "iCub::l_hand::l_hand_collision");

  REQUIRE(force.x == Approx(3.0));
  REQUIRE(force.y == Approx(0.0));
  REQUIRE(force.z == Approx(0.0));
}

TEST_CASE("AggregateGuiForces identifies external contact body", "[contact_utils]") {
  vectorview::ContactForce contact;
  vectorview::WrenchForce wrench;
  wrench.body_1_name = "iCub::l_hand::l_hand_collision";
  wrench.body_2_name = "box::link::collision";
  wrench.body_1_force = vectorview::Vec3(2.0, 0.0, 0.0);
  wrench.body_2_force = vectorview::Vec3(0.0, 0.0, 0.0);
  contact.wrenches.push_back(wrench);

  std::vector<vectorview::ContactForce> contacts(1, contact);
  const vectorview::GuiContactResult result =
      vectorview::AggregateGuiForces(contacts, "iCub");

  REQUIRE(result.has_wrenches);
  REQUIRE(result.object_name == "box::link::collision");
  REQUIRE(result.force.x == Approx(2.0));
}
