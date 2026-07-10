#ifndef VECTORVIEW_TOPIC_PATH_H
#define VECTORVIEW_TOPIC_PATH_H

#include "vectorview/ModelContext.h"

#include <string>

namespace vectorview {

struct TopicPath {
  std::string transport;
  std::string collision_scope;
  bool valid;

  TopicPath() : valid(false) {}

  static TopicPath FromVisualName(const std::string& visual_name);
  static TopicPath FromCliArgument(const std::string& argument, const ModelContext& context);
};

}  // namespace vectorview

#endif
