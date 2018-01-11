#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "point_type.h"

namespace pc {
  class PointCloud {
  public:
    typedef std::shared_ptr<PointCloud> Ptr;

    PointCloud() {}

    PointCloud(const Eigen::Matrix3Xf& matrix) { //Matrix3Xf to PointCloud
      if(matrix.cols()) {
        pts.resize(matrix.cols());
        for (size_t i = 0; i < matrix.cols(); i++) {
          pts.at(i) = matrix.col(i);
        }
      }
    }

    PointCloud(const Eigen::MatrixX3f& matrix) { //MatrixX3f to PointCloud
      if(matrix.rows()) {
        pts.resize(matrix.rows());
        for (size_t i = 0; i < matrix.rows(); i++) {
          pts.at(i) = matrix.row(i);
        }
      }
    }

    inline PointCloud& operator += (PointCloud& rhs) {
      size_t nr_pts = pts.size();
  		pts.resize(nr_pts + rhs.pts.size());
  		for (size_t i = nr_pts; i < pts.size(); ++i)
  			pts[i] = rhs.pts[i - nr_pts];
  		return *this;
    }

    inline PointCloud operator + (PointCloud& rhs) {
      return (PointCloud(*this) += rhs);
    }

    typedef std::vector<PointNormal>::iterator iterator;
	  typedef std::vector<PointNormal>::const_iterator const_iterator;

<<<<<<< HEAD
    iterator begin () {
      return pts.begin();
    }

    iterator end() {
      return pts.end();
    }

    const_iterator begin() const {
      return pts.begin();
    }

    const_iterator end() const {
      return pts.end();
    }

    size_t size() const {
      return pts.size();
    }

    size_t capacity() const {
      return pts.capacity();
    }

    void reserve(size_t n) {
      pts.reserve(n);
    }

    bool empty() const {
      return pts.empty();
    }

    void resize(size_t n) {
      pts.resize(n);
    }

    const PointNormal& operator[](size_t n) const {
      return pts[n];
    }

    PointNormal& operator[](size_t n) {
      return pts[n];
    }

    const PointNormal& at(size_t n) const {
      return pts.at(n);
    }

    PointNormal& at(size_t n) {
      return pts.at(n);
    }

    const PointNormal& front () const {
      return pts.front();
    }

    PointNormal& front() {
      return pts.front ();
    }

    const PointNormal& back() const {
      return pts.back ();
    }

    PointNormal& back() {
      return pts.back ();
    }

    void push_back (const PointNormal& pt) {
      pts.push_back(pt);
    }

    iterator insert(iterator position, const PointNormal& pt) {
=======
    inline iterator begin () {
      return pts.begin();
    }

    inline iterator end() {
      return pts.end();
    }
    inline const_iterator begin() const {
      return pts.begin();
    }

    inline const_iterator end() const {
      return pts.end();
    }

    inline size_t size() const {
      return pts.size();
    }

    inline void reserve(size_t n) {
      pts.reserve(n);
    }

    inline bool empty() const {
      return pts.empty();
    }

    inline void resize(size_t n) {
      pts.resize(n);
    }

    inline const PointNormal& operator[](size_t n) const {
      return pts[n];
    }

    inline PointNormal& operator[](size_t n) {
      return pts[n];
    }

    inline const PointNormal& at(size_t n) const {
      return pts.at(n);
    }

    inline PointNormal& at(size_t n) {
      return pts.at(n);
    }

    inline const PointNormal& front () const {
      return pts.front();
    }

    inline PointNormal& front() {
      return pts.front ();
    }

    inline const PointNormal& back() const {
      return pts.back ();
    }

    inline PointNormal& back() {
      return pts.back ();
    }

    inline void push_back (const PointNormal& pt) {
      pts.push_back(pt);
    }

    inline iterator insert(iterator position, const PointNormal& pt) {
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
      iterator it = pts.insert (position, pt);
      return it;
    }

<<<<<<< HEAD
    iterator erase(iterator position) {
=======
    inline iterator erase(iterator position) {
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
      iterator it = pts.erase (position);
      return it;
    }

<<<<<<< HEAD
    iterator erase (iterator first, iterator last) {
=======
    inline iterator erase (iterator first, iterator last) {
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
      iterator it = pts.erase (first, last);
      return it;
    }

<<<<<<< HEAD
    void swap (PointCloud &rhs) {
      this->pts.swap (rhs.pts);
    }

    void clear () {
=======
    inline void swap (PointCloud &rhs) {
      this->pts.swap (rhs.pts);
    }

    inline void clear () {
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
      pts.clear ();
    }

    PointCloud mean() {

    }

  private:
    std::vector<PointNormal> pts;
  };

<<<<<<< HEAD
  std::ostream& operator << (std::ostream& os, const PointCloud& pc);
=======
  std::ostream& operator << (std::ostream& os, const PointCloud& pc) {
    os << "points size: " << pc.size() << '\n'
       << "x y z nx ny nz" << '\n';
    for (size_t i = 0; i < pc.size(); i++)
      os << pc.at(i) << '\n';
    return os;
  }
>>>>>>> 756abd7752b21a06312a4b716e561f463f747b43
}

#endif
