#include "catch.hpp"

#include "vectorview/ContactUtils.h"
#include "vectorview/ModelContext.h"
#include "vectorview/TopicPath.h"

TEST_CASE("TopicPath derives link-based topics from visual names", "[topic_path]") {
  const vectorview::TopicPath path =
      vectorview::TopicPath::FromVisualName("iCub::l_hand::l_hand_visual");

  REQUIRE(path.valid);
  REQUIRE(path.transport == "/vectorview/iCub_fixed/l_hand");
  REQUIRE(path.collision_scope == "iCub::l_hand::l_hand_collision");
}

TEST_CASE("TopicPath handles single-segment visual names", "[topic_path]") {
  const vectorview::TopicPath path = vectorview::TopicPath::FromVisualName("l_hand_visual");

  REQUIRE(path.valid);
  REQUIRE(path.transport == "/vectorview/iCub_fixed/l_hand");
  REQUIRE(path.collision_scope == "l_hand::l_hand_collision");
}

TEST_CASE("TopicPath rejects empty visual names", "[topic_path]") {
  const vectorview::TopicPath path = vectorview::TopicPath::FromVisualName("");
  REQUIRE_FALSE(path.valid);
}

TEST_CASE("TopicPath builds GUI topics from short link names", "[topic_path]") {
  const vectorview::ModelContext context;
  const vectorview::TopicPath path = vectorview::TopicPath::FromCliArgument("l_hand", context);

  REQUIRE(path.valid);
  REQUIRE(path.transport == "/vectorview/iCub_fixed/l_hand");
}

TEST_CASE("TopicPath accepts full transport paths unchanged", "[topic_path]") {
  const vectorview::ModelContext context;
  const vectorview::TopicPath path = vectorview::TopicPath::FromCliArgument(
      "/vectorview/iCub_fixed/r_hand", context);

  REQUIRE(path.valid);
  REQUIRE(path.transport == "/vectorview/iCub_fixed/r_hand");
}

TEST_CASE("TopicPath strips _contact suffix from short names", "[topic_path]") {
  const vectorview::ModelContext context;
  const vectorview::TopicPath path =
      vectorview::TopicPath::FromCliArgument("l_hand_contact", context);

  REQUIRE(path.valid);
  REQUIRE(path.transport == "/vectorview/iCub_fixed/l_hand");
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
