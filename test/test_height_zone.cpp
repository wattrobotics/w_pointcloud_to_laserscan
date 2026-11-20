// Unit tests for height_zone::decide / normalizeFloor (pure logic, no ROS).
#include <gtest/gtest.h>

#include <vector>

#include "pointcloud_to_laserscan/height_zone.hpp"

using pointcloud_to_laserscan::height_zone::Observation;
using pointcloud_to_laserscan::height_zone::Pose;
using pointcloud_to_laserscan::height_zone::Zone;
using pointcloud_to_laserscan::height_zone::decide;
using pointcloud_to_laserscan::height_zone::normalizeFloor;

namespace
{
Zone station()
{
  Zone z;
  z.name = "near_station";
  z.profile = "station";
  z.radius = 3.0;
  z.exit_margin = 0.3;
  z.resolved = true;
  z.x = 1.8;
  z.y = 1.538;
  z.floor = "B1";
  return z;
}

Observation at(double x, double y, const char * floor = "B1", bool pose_ok = true)
{
  Observation o;
  o.poses = {Pose{pose_ok, x, y}};
  o.floor_ok = floor != nullptr;
  o.floor = floor ? floor : "";
  return o;
}
}  // namespace

TEST(HeightZone, NormalizeFloor)
{
  EXPECT_EQ(normalizeFloor("b1"), "B1");
  EXPECT_EQ(normalizeFloor("B1F"), "B1");
  EXPECT_EQ(normalizeFloor(" 1F "), "1");
  EXPECT_EQ(normalizeFloor("01"), "1");
  EXPECT_EQ(normalizeFloor("L"), "L");
  EXPECT_EQ(normalizeFloor("F"), "F");
  EXPECT_EQ(normalizeFloor(""), "");
}

TEST(HeightZone, EnterInsideRadiusOnSameFloor)
{
  const std::vector<Zone> zs{station()};
  auto d = decide(zs, -1, at(1.8 + 2.9, 1.538));
  EXPECT_EQ(d.zone, 0);
  EXPECT_NEAR(d.distance, 2.9, 1e-9);
}

TEST(HeightZone, HysteresisBandKeepsCurrent)
{
  const std::vector<Zone> zs{station()};
  const auto o = at(1.8 + 3.15, 1.538);   // radius < d <= radius + margin
  EXPECT_EQ(decide(zs, -1, o).zone, -1);  // outside stays outside
  EXPECT_EQ(decide(zs, 0, o).zone, 0);    // inside stays inside
}

TEST(HeightZone, LeaveBeyondExitMargin)
{
  const std::vector<Zone> zs{station()};
  auto d = decide(zs, 0, at(1.8 + 3.31, 1.538));
  EXPECT_EQ(d.zone, -1);
  EXPECT_NEAR(d.distance, 3.31, 1e-9);
}

TEST(HeightZone, OtherFloorNeverEntersAndLeaves)
{
  const std::vector<Zone> zs{station()};
  EXPECT_EQ(decide(zs, -1, at(1.8, 1.538, "1")).zone, -1);
  EXPECT_EQ(decide(zs, 0, at(1.8, 1.538, "1")).zone, -1);
}

TEST(HeightZone, UnknownFloor)
{
  const std::vector<Zone> zs{station()};
  EXPECT_EQ(decide(zs, -1, at(1.8, 1.538, nullptr)).zone, -1);        // no entry
  EXPECT_EQ(decide(zs, 0, at(1.8 + 1.0, 1.538, nullptr)).zone, 0);    // stay inside
  EXPECT_EQ(decide(zs, 0, at(1.8 + 5.0, 1.538, nullptr)).zone, -1);   // pose says far -> leave
}

TEST(HeightZone, UnknownPoseKeepsLast)
{
  const std::vector<Zone> zs{station()};
  EXPECT_EQ(decide(zs, 0, at(99, 99, "1", false)).zone, 0);
  EXPECT_EQ(decide(zs, -1, at(1.8, 1.538, "B1", false)).zone, -1);
  Observation none;  // no poses at all
  EXPECT_EQ(decide(zs, 0, none).zone, 0);
}

TEST(HeightZone, UnresolvedZoneIsSkipped)
{
  auto z = station();
  z.resolved = false;
  const std::vector<Zone> zs{z};
  EXPECT_EQ(decide(zs, -1, at(1.8, 1.538)).zone, -1);
}

TEST(HeightZone, NearestOfTwoAndDirectSwitch)
{
  auto a = station();
  auto b = station();
  b.name = "b";
  b.x = 1.8 + 4.0;
  const std::vector<Zone> zs{a, b};
  Observation o;
  o.poses = {Pose{true, 1.8 + 2.5, 1.538}, Pose{true, 1.8 + 2.5, 1.538}};
  o.floor_ok = true;
  o.floor = "B1";
  EXPECT_EQ(decide(zs, -1, o).zone, 1);  // 1.5 m to b vs 2.5 m to a
  o.poses = {Pose{true, 1.8 + 3.5, 1.538}, Pose{true, 1.8 + 3.5, 1.538}};
  EXPECT_EQ(decide(zs, 0, o).zone, 1);   // left a (3.5 > 3.3), inside b
}
