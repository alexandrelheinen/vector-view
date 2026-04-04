#include <gtest/gtest.h>
#include "vectorview/NameUtils.h"

using VectorView::ParseVisualName;

TEST(ParseVisualName, EmptyNameReturnsFalse)
{
  std::string topic, collision;
  EXPECT_FALSE(ParseVisualName("", topic, collision));
}

TEST(ParseVisualName, FlatNameNoScope)
{
  std::string topic, collision;
  ASSERT_TRUE(ParseVisualName("l_hand", topic, collision));
  EXPECT_EQ(topic,     "~/l_hand/l_hand_contact");
  EXPECT_EQ(collision, "l_hand::l_hand_collision");
}

TEST(ParseVisualName, ThreeSegmentName)
{
  std::string topic, collision;
  ASSERT_TRUE(ParseVisualName("robot::iCub::r_hand", topic, collision));
  EXPECT_EQ(topic,     "~/robot/iCub/r_hand/r_hand_contact");
  EXPECT_EQ(collision, "robot::iCub::r_hand::r_hand_collision");
}

TEST(ParseVisualName, TwoSegmentName)
{
  std::string topic, collision;
  ASSERT_TRUE(ParseVisualName("iCub_fixed::r_hand", topic, collision));
  EXPECT_EQ(topic,     "~/iCub_fixed/r_hand/r_hand_contact");
  EXPECT_EQ(collision, "iCub_fixed::r_hand::r_hand_collision");
}
