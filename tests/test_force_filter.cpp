#include "catch.hpp"

#include "vectorview/Constants.h"
#include "vectorview/ForceFilter.h"

TEST_CASE("ForceFilter passes constant input through", "[force_filter]") {
  vectorview::ForceFilter filter;
  vectorview::Vec3 force(1.0, 0.0, 0.0);

  const double original_length = filter.Filter(&force);

  REQUIRE(original_length == Approx(1.0));
  REQUIRE(force.x == Approx(1.0).margin(0.2));
}

TEST_CASE("ForceFilter returns original magnitude", "[force_filter]") {
  vectorview::ForceFilter filter;
  vectorview::Vec3 force(3.0, 4.0, 0.0);

  const double original_length = filter.Filter(&force);

  REQUIRE(original_length == Approx(5.0));
}

TEST_CASE("ForceFilter handles zero input", "[force_filter]") {
  vectorview::ForceFilter filter;
  vectorview::Vec3 force(0.0, 0.0, 0.0);

  const double original_length = filter.Filter(&force);

  REQUIRE(original_length == Approx(0.0));
  REQUIRE(force.Length() == Approx(0.0));
}

TEST_CASE("Shared constants stay aligned with sensor rate", "[constants]") {
  REQUIRE(RATE == 25);
  REQUIRE(NOISE_THRESHOLD == Approx(1E-3));
}
