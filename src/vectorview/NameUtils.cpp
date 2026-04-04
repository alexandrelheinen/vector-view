#include "vectorview/NameUtils.h"
#include <vector>

namespace VectorView
{

bool ParseVisualName(const std::string& visualName,
                     std::string& topicName,
                     std::string& collisionName)
{
  if (visualName.empty())
    return false;

  std::vector<std::string> segments;
  std::string name = visualName;

  while (name.find("::") != std::string::npos)
  {
    segments.push_back(name.substr(0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  segments.push_back(name);  // push the final segment

  topicName     = "~";
  collisionName = "";

  for (size_t i = 0; i < segments.size(); ++i)
  {
    topicName     += "/" + segments[i];
    collisionName += "::" + segments[i];
  }
  topicName     += "/" + segments.back() + "_contact";
  collisionName += "::" + segments.back() + "_collision";
  collisionName.erase(0, 2);

  return true;
}

}  // namespace VectorView
