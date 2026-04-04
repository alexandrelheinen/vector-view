#pragma once
#include <string>

namespace VectorView
{
  /**
   * @brief Derives a Gazebo contact-sensor topic name and collision body name
   *        from a scoped Gazebo visual name.
   *
   * The visual name is expected to be a double-colon–separated scope path,
   * e.g. @c "robot::link::visual".  The function splits it into segments,
   * then builds:
   * @li @p topicName      – @c "~/seg0/seg1/.../segN/segN_contact"
   * @li @p collisionName  – @c "seg0::seg1::...::segN::segN_collision"
   *
   * @param visualName    Scoped Gazebo visual name.
   * @param topicName     Output: Gazebo transport topic path.
   * @param collisionName Output: Collision body name.
   * @return @c true on success; @c false if @p visualName is empty.
   */
  bool ParseVisualName(const std::string& visualName,
                       std::string& topicName,
                       std::string& collisionName);
}
