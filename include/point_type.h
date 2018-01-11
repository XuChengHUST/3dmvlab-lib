#ifndef POINT_TYPE_H
#define POINT_TYPE_H

#include "common_include.h"

namespace pc {
  struct PointNormal {
    float x, y, z, nx, ny, nz;

    PointNormal() :
      x(0.0f), y(0.0f), z(0.0f), nx(0.0f), ny(0.0f), nz(0.0f) {}

    PointNormal(const PointNormal& pt) {
      x = pt.x; y = pt.y; z = pt.z;
      nx = pt.nx; ny = pt.ny; nz = pt.nz;
    }

    PointNormal(float _x, float _y, float _z,
                float _nx = 0.0f, float _ny = 0.0f, float _nz = 0.0f) {
      x = _x; y = _y; z = _z;
      nx = _nx; ny = _ny; nz = _nz;
    }

    PointNormal(const Eigen::Vector3f& pt) {
      x = pt(0), y = pt(1), z = pt(2);
      nx = 0.0f; ny = 0.0f; nz = 0.0f;
    }

    PointNormal(const Eigen::Vector3f& pt, const Eigen::Vector3f& normal) {
      x = pt(0), y = pt(1), z = pt(2);
      nx = normal(0); ny = normal(1); nz = normal(2);
    }

    PointNormal(const Eigen::Vector3d& pt, const Eigen::Vector3d& normal) {
      x = pt(0), y = pt(1), z = pt(2);
      nx = normal(0); ny = normal(1); nz = normal(2);
    }

    inline PointNormal operator=(const Eigen::Vector3f& rhs) {
      return PointNormal(rhs);
    }
  };

<<<<<<< HEAD
  std::ostream& operator << (std::ostream& os, const PointNormal& pt);
=======
  std::ostream& operator << (std::ostream& os, const PointNormal& pt) {
    os << pt.x << " " << pt.y << " " << pt.z << " "
       << pt.nx << " " << pt.ny << " " << pt.nz;
    return os;
  }
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
}

#endif
