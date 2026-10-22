// Sparse near-field ghost filter for the projected LaserScan (header-only, no ROS deps).
//
// A ghost here is a sparse point floating just in front of a dense surface. Each projectable
// slab point within max_check_range (horizontal) of the sensor center is "supported" when at
// least min_neighbors other slab points lie within max(min_radius, radius_ratio * r3), where r3
// is its 3D distance to the center (the radius grows with range like the beam spacing does).
// Per bin: if the nearest point is unsupported and the nearest supported point of the same bin
// is at most behind_distance farther, the bin takes the supported range. Otherwise the bin is
// left as it is, so a bin never becomes empty and never moves back by more than behind_distance.

#ifndef POINTCLOUD_TO_LASERSCAN__GHOST_FILTER_HPP_
#define POINTCLOUD_TO_LASERSCAN__GHOST_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace pointcloud_to_laserscan
{

struct GhostFilterParams
{
  double center_x = 0.0, center_y = 0.0, center_z = 0.0;  // sensor origin in the scan frame
  double radius_ratio = 0.10;
  double min_radius = 0.02;
  int min_neighbors = 8;
  double max_check_range = 1.5;
  double behind_distance = 0.15;
};

class GhostFilter
{
public:
  explicit GhostFilter(const GhostFilterParams & p)
  : p_(p) {}

  void clear()
  {
    pool_.clear();
    cand_.clear();
  }

  // Every slab point (after NaN/tag/height filters, before self-box/range/angle filters).
  void addPoolPoint(double x, double y, double z)
  {
    if (std::hypot(x - p_.center_x, y - p_.center_y) <= poolRange()) {
      pool_.push_back({x, y, z});
    }
  }

  // A point that went into the scan: its bin and range.
  void addScanPoint(double x, double y, double z, int bin, double range)
  {
    cand_.push_back({x, y, z, bin, range});
  }

  // ranges holds the nearest range per bin (unfiltered). Returns the number of bins changed.
  std::size_t apply(std::vector<float> & ranges)
  {
    buildGrid();
    std::vector<double> supported(ranges.size(), std::numeric_limits<double>::infinity());
    for (const auto & c : cand_) {
      if (c.bin < 0 || c.bin >= static_cast<int>(ranges.size())) {continue;}
      if (c.range < supported[c.bin] && isSupported(c)) {
        supported[c.bin] = c.range;
      }
    }
    std::size_t changed = 0;
    for (std::size_t b = 0; b < ranges.size(); ++b) {
      const double s = supported[b];
      if (std::isfinite(s) && s > ranges[b] && s <= ranges[b] + p_.behind_distance) {
        ranges[b] = static_cast<float>(s);
        ++changed;
      }
    }
    return changed;
  }

private:
  struct P3 {double x, y, z;};
  struct Cand {double x, y, z; int bin; double range;};
  static constexpr double kCell = 0.05;

  double maxRadius() const {return std::max(p_.min_radius, p_.radius_ratio * (p_.max_check_range + 0.5));}
  double poolRange() const {return p_.max_check_range + maxRadius();}
  static int64_t key(int ix, int iy) {return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);}

  void buildGrid()
  {
    grid_.clear();
    for (std::size_t i = 0; i < pool_.size(); ++i) {
      const int ix = static_cast<int>(std::floor(pool_[i].x / kCell));
      const int iy = static_cast<int>(std::floor(pool_[i].y / kCell));
      grid_[key(ix, iy)].push_back(static_cast<uint32_t>(i));
    }
  }

  bool isSupported(const Cand & c) const
  {
    const double dx0 = c.x - p_.center_x, dy0 = c.y - p_.center_y, dz0 = c.z - p_.center_z;
    if (std::hypot(dx0, dy0) > p_.max_check_range) {return true;}  // not checked
    const double r3 = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
    const double rad = std::max(p_.min_radius, p_.radius_ratio * r3);
    const double rad2 = rad * rad;
    const int need = p_.min_neighbors + 1;  // the point itself is in the pool
    const int ix0 = static_cast<int>(std::floor((c.x - rad) / kCell));
    const int ix1 = static_cast<int>(std::floor((c.x + rad) / kCell));
    const int iy0 = static_cast<int>(std::floor((c.y - rad) / kCell));
    const int iy1 = static_cast<int>(std::floor((c.y + rad) / kCell));
    int count = 0;
    for (int ix = ix0; ix <= ix1; ++ix) {
      for (int iy = iy0; iy <= iy1; ++iy) {
        const auto it = grid_.find(key(ix, iy));
        if (it == grid_.end()) {continue;}
        for (const uint32_t j : it->second) {
          const double dx = pool_[j].x - c.x, dy = pool_[j].y - c.y, dz = pool_[j].z - c.z;
          if (dx * dx + dy * dy + dz * dz <= rad2 && ++count >= need) {return true;}
        }
      }
    }
    return false;
  }

  GhostFilterParams p_;
  std::vector<P3> pool_;
  std::vector<Cand> cand_;
  std::unordered_map<int64_t, std::vector<uint32_t>> grid_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__GHOST_FILTER_HPP_
