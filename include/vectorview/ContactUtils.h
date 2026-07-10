#ifndef VECTORVIEW_CONTACT_UTILS_H
#define VECTORVIEW_CONTACT_UTILS_H

#include <cmath>
#include <string>
#include <vector>

namespace vectorview {

struct TopicNames {
  std::string topic;
  std::string collision;
  bool valid;

  TopicNames() : valid(false) {}
};

struct Vec3 {
  double x;
  double y;
  double z;

  Vec3() : x(0.0), y(0.0), z(0.0) {}
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& other) const {
    return Vec3(x + other.x, y + other.y, z + other.z);
  }

  Vec3& operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Vec3 operator/(double divisor) const {
    return Vec3(x / divisor, y / divisor, z / divisor);
  }

  double Length() const { return std::sqrt(x * x + y * y + z * z); }
};

struct WrenchForce {
  std::string body_1_name;
  std::string body_2_name;
  Vec3 body_1_force;
  Vec3 body_2_force;
};

struct ContactForce {
  std::vector<WrenchForce> wrenches;
};

struct GuiContactResult {
  Vec3 force;
  std::string object_name;
  bool has_wrenches;
};

TopicNames DeriveTopicNames(const std::string& visual_name);

Vec3 AggregatePluginForces(const std::vector<ContactForce>& contacts,
                           const std::string& collision_name);

GuiContactResult AggregateGuiForces(const std::vector<ContactForce>& contacts,
                                    const std::string& robot_name);

}  // namespace vectorview

#endif
