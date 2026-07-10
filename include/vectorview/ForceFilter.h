#ifndef VECTORVIEW_FORCE_FILTER_H
#define VECTORVIEW_FORCE_FILTER_H

#include "vectorview/Constants.h"
#include "vectorview/ContactUtils.h"

#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"

#include <memory>

namespace vectorview {

class ForceFilter {
 public:
  ForceFilter();
  ~ForceFilter();

  double Filter(Vec3* vector);

 private:
  std::unique_ptr<Dsp::Filter> filter;
};

}  // namespace vectorview

#endif
