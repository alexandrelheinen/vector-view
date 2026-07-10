#ifndef VECTORVIEW_MODEL_CONTEXT_H
#define VECTORVIEW_MODEL_CONTEXT_H

#include <string>

namespace vectorview {

struct ModelContext {
  std::string world_name = "default";
  std::string model_instance = "iCub_fixed";
  std::string robot_model = "iCub";
};

}  // namespace vectorview

#endif
