#include <gtest/gtest.h>
#include <ignition/math/Vector3.hh>
#include "DspFilters/ForceFilter.h"

TEST(ForceFilter, ConstructionDoesNotThrow)
{
  EXPECT_NO_THROW(Dsp::ForceFilter f);
}

TEST(ForceFilter, ZeroInputStaysNearZero)
{
  Dsp::ForceFilter f;
  // Feed zero vector; output should remain near zero.
  for (int i = 0; i < 100; ++i)
  {
    ignition::math::Vector3d v = ignition::math::Vector3d::Zero;
    f.Filter(&v);
    EXPECT_NEAR(v.X(), 0.0, 1e-12);
    EXPECT_NEAR(v.Y(), 0.0, 1e-12);
    EXPECT_NEAR(v.Z(), 0.0, 1e-12);
  }
}

TEST(ForceFilter, StepInputAttenuatedAtHighFrequency)
{
  // Feed a constant unit-X step to the 1.5 Hz cut-off filter.
  // After enough samples the output should converge toward 1.
  Dsp::ForceFilter f;
  ignition::math::Vector3d v;
  double lastX = 0.0;
  for (int i = 0; i < 500; ++i)
  {
    v = ignition::math::Vector3d(1.0, 0.0, 0.0);
    f.Filter(&v);
    lastX = v.X();
  }
  // Steady-state response of a low-pass filter to a step is 1.
  EXPECT_NEAR(lastX, 1.0, 1e-3);
}

TEST(ForceFilter, FilterReturnsPrefilteredLength)
{
  Dsp::ForceFilter f;
  ignition::math::Vector3d v(3.0, 4.0, 0.0);  // length == 5.0
  double prelen = f.Filter(&v);
  EXPECT_NEAR(prelen, 5.0, 1e-9);
}
