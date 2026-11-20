// Position-based height-profile zones (pure logic, no ROS) — see PointCloudToLaserScanNode.
// A zone is a circle around a robot-DB site point. Inside `radius` on the same floor -> the
// zone's profile; beyond `radius + exit_margin` (or on another known floor) -> default;
// in between -> keep. Unknown pose of the active zone -> keep. Unknown floor -> no entry,
// but an already-active zone stays while within radius + exit_margin.

#ifndef POINTCLOUD_TO_LASERSCAN__HEIGHT_ZONE_HPP_
#define POINTCLOUD_TO_LASERSCAN__HEIGHT_ZONE_HPP_

#include <cctype>
#include <cmath>
#include <string>
#include <vector>

namespace pointcloud_to_laserscan
{
namespace height_zone
{

struct Zone
{
  std::string name;
  std::string point_code;
  std::string profile;
  std::string frame{"map"};
  double radius{3.0};
  double exit_margin{0.3};
  // Filled by the DB lookup. Unresolved zones are never entered.
  bool resolved{false};
  double x{0.0}, y{0.0};
  std::string floor;  // normalised
};

// Robot position in one zone's frame.
struct Pose
{
  bool ok{false};
  double x{0.0}, y{0.0};
};

struct Observation
{
  std::vector<Pose> poses;  // one per zone (same order); missing/!ok = TF unavailable
  bool floor_ok{false};
  std::string floor;        // normalised
};

struct Decision
{
  int zone{-1};           // index into zones, -1 = default profile
  double distance{NAN};   // distance to the entered / kept / left zone (NaN if none)
};

// "b1f" -> "B1", "1F" -> "1", "01" -> "1", " L " -> "L". Empty stays empty (= unknown).
inline std::string normalizeFloor(const std::string & in)
{
  std::string s;
  for (char c : in) {
    if (!std::isspace(static_cast<unsigned char>(c))) {
      s += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
  }
  if (s.size() > 1 && s.back() == 'F') {
    s.pop_back();
  }
  size_t i = (!s.empty() && s[0] == 'B') ? 1 : 0;
  while (s.size() > i + 1 && s[i] == '0') {
    s.erase(i, 1);
  }
  return s;
}

// `active` = currently active zone index or -1.
inline Decision decide(const std::vector<Zone> & zones, int active, const Observation & obs)
{
  auto pose = [&obs](size_t i) {return i < obs.poses.size() ? obs.poses[i] : Pose{};};
  Decision left{-1, NAN};
  if (active >= 0 && active < static_cast<int>(zones.size())) {
    const Zone & z = zones[active];
    const Pose p = pose(active);
    if (!z.resolved || !p.ok) {
      return {active, NAN};  // cannot tell -> keep
    }
    const double d = std::hypot(p.x - z.x, p.y - z.y);
    const bool other_floor = obs.floor_ok && obs.floor != z.floor;
    if (!other_floor && d <= z.radius + z.exit_margin) {
      return {active, d};
    }
    left.distance = d;
  }
  // Entry: nearest resolved zone on the (known) current floor within its radius.
  Decision best = left;
  double best_d = INFINITY;
  if (obs.floor_ok) {
    for (size_t i = 0; i < zones.size(); ++i) {
      const Zone & z = zones[i];
      const Pose p = pose(i);
      if (!z.resolved || !p.ok || z.floor != obs.floor) {
        continue;
      }
      const double d = std::hypot(p.x - z.x, p.y - z.y);
      if (d <= z.radius && d < best_d) {
        best_d = d;
        best = {static_cast<int>(i), d};
      }
    }
  }
  return best;
}

}  // namespace height_zone
}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__HEIGHT_ZONE_HPP_
