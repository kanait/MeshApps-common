////////////////////////////////////////////////////////////////////
//
// $Id: myEigen.hxx 2022/05/14 21:40:03 kanai Exp $
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MYEIGEN_HXX
#define _MYEIGEN_HXX 1

#include <cmath>
//using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>
typedef Eigen::Triplet<double> T;

// return (double)Math.acos(dot(v1)/v1.length()/v.length());
// Numerically, near 0 and PI are very bad condition for acos.
// In 3-space, |atan2(sin,cos)| is much stable.
template <typename T>
T angleT( Eigen::Matrix<T, 3, 1>& v1, Eigen::Matrix<T, 3, 1>& v2 ) {
  Eigen::Matrix<T, 3, 1> c = v1.cross(v2);
  T s = c.norm();

  return std::fabs(std::atan2(s, v1.dot(v2)));
};

//typedef angleT<float> anglef;

#if 0
float anglef( Eigen::Vector3f& v1, Eigen::Vector3f& v2 ) {
  // return (double)Math.acos(dot(v1)/v1.length()/v.length());
  // Numerically, near 0 and PI are very bad condition for acos.
  // In 3-space, |atan2(sin,cos)| is much stable.
  Eigen::Vector3f c = v1.cross(v2);
  float s = c.norm();

  return std::fabs(std::atan2(s, v1.dot(v2)));
};
#endif


#endif // _MYEIGEN_HXX

