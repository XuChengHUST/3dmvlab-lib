#include "point_type.h"
#include "point_cloud.h"

namespace pc {
  std::ostream& operator << (std::ostream& os, const PointNormal& pt) {
    os << pt.x << " " << pt.y << " " << pt.z << " "
       << pt.nx << " " << pt.ny << " " << pt.nz;
    return os;
  }

  std::ostream& operator << (std::ostream& os, const PointCloud& pc) {
    os << "points size: " << pc.size() << '\n'
       << "x y z nx ny nz" << '\n';
    for (size_t i = 0; i < pc.size(); i++)
      os << pc.at(i) << '\n';
    return os;
  }
}
