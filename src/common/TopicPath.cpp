#include "vectorview/TopicPath.h"

#include <vector>

namespace vectorview {
namespace {

std::vector<std::string> SplitScopedName(const std::string& scoped_name) {
  std::vector<std::string> segments;
  std::string remainder = scoped_name;

  while (remainder.find("::") != std::string::npos) {
    const size_t delimiter = remainder.find("::");
    segments.push_back(remainder.substr(0, delimiter));
    remainder.erase(0, delimiter + 2);
  }

  if (!remainder.empty()) {
    segments.push_back(remainder);
  }

  return segments;
}

std::string ExtractLinkName(const std::string& visual_segment) {
  const std::string suffix = "_visual";
  if (visual_segment.size() > suffix.size() &&
      visual_segment.compare(visual_segment.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return visual_segment.substr(0, visual_segment.size() - suffix.size());
  }
  return visual_segment;
}

std::string BuildTransportTopic(const std::vector<std::string>& segments,
                                const std::string& link_name) {
  std::string topic = "~";
  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    topic += "/" + segments[i];
  }
  topic += "/" + link_name + "/" + link_name + "_contact";
  return topic;
}

std::string BuildCollisionScope(const std::vector<std::string>& segments,
                                const std::string& link_name) {
  std::string collision;
  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    if (!collision.empty()) {
      collision += "::";
    }
    collision += segments[i];
  }

  if (!collision.empty()) {
    collision += "::";
  }
  collision += link_name + "_collision";
  return collision;
}

}  // namespace

TopicPath TopicPath::FromVisualName(const std::string& visual_name) {
  TopicPath result;
  if (visual_name.empty()) {
    return result;
  }

  const std::vector<std::string> segments = SplitScopedName(visual_name);
  if (segments.empty()) {
    return result;
  }

  const std::string link_name = ExtractLinkName(segments.back());
  if (link_name.empty()) {
    return result;
  }

  result.transport = BuildTransportTopic(segments, link_name);
  result.collision_scope = BuildCollisionScope(segments, link_name);
  result.valid = true;
  return result;
}

TopicPath TopicPath::FromCliArgument(const std::string& argument, const ModelContext& context) {
  TopicPath result;
  if (argument.empty()) {
    return result;
  }

  if (argument.find('/') != std::string::npos) {
    result.transport = argument;
    result.valid = true;
    return result;
  }

  std::string link_name = argument;
  const std::string contact_suffix = "_contact";
  if (link_name.size() > contact_suffix.size() &&
      link_name.compare(link_name.size() - contact_suffix.size(), contact_suffix.size(),
                        contact_suffix) == 0) {
    link_name.erase(link_name.size() - contact_suffix.size());
  }

  if (link_name.empty()) {
    return result;
  }

  result.transport = "/gazebo/" + context.world_name + "/" + context.model_instance + "/" +
                     context.robot_model + "/" + link_name + "/" + link_name + "_contact";
  result.valid = true;
  return result;
}

}  // namespace vectorview
