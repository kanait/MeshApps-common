////////////////////////////////////////////////////////////////////
//
// $Id: VMProc.hxx 2025/07/19 14:11:45 kanai Exp 
//
// Copyright (c) 2002-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _VMPROC_HXX
#define _VMPROC_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
//using namespace std;

#include "myEigen.hxx"

template <typename Type, int Size> using Vector = Eigen::Matrix<Type, Size, 1>;

template <class T>
inline void calcNormal( Eigen::Vector<T,3>&, Eigen::Vector<T,3>&, Eigen::Vector<T,3>&, Eigen::Vector<T,3>& );

template <class T>
class Plane {

public:

  Plane( Eigen::Vector<T,3>& a, Eigen::Vector<T,3>& b, Eigen::Vector<T,3>& c ) {
    Eigen::Vector<T,3> n;
    calcNormal<T>( a, b, c, n );
    a_ = n.x(); b_ = n.y(); c_ = n.z();
    d_ = -1.0*( a_*a.x() + b_*a.y() + c_*a.z() );
  };

  Plane( Eigen::Vector<T,3>& p, Eigen::Vector<T,3>& n ) {
    //n.normalize();
    a_ = n.x(); b_ = n.y(); c_ = n.z();
    d_ = -1.0*( a_*p.x() + b_*p.y() + c_*p.z() );
  };
  ~Plane(){};

  bool isLeftSide( Eigen::Vector<T,3>& p ) {
    return ( (a_*p.x()+b_*p.y()+c_*p.z()+d_) > .0 ) ? true : false;
  };

  T signedDistance( Eigen::Vector<T,3>& p ) {
    return a_*p.x()+b_*p.y()+c_*p.z()+d_;
  };

  // T signedDistance( Eigen::Vector<T,3>& p ) {
  //   return a_*p.x()+b_*p.y()+c_*p.z()+d_;
  // };

  T distance( Eigen::Vector<T,3>& p ) {
    return std::fabs( a_*p.x()+b_*p.y()+c_*p.z()+d_);
  };

  //
  // p, h は絶対位置
  //
  void calcPointToPlane( Eigen::Vector<T,3>& p, Eigen::Vector<T,3>& h )
  {
    Eigen::Vector<T,3> n( a_, b_, c_ ); //cout << n << endl;
    Eigen::Vector<T,3> nn( n ); nn *= ( n.dot(p) + d_ );
    h = p - nn;
//      cout << "dist p: " << signedDistance( p ) << endl;
//      cout << "dist h: " << signedDistance( h ) << endl;
  };

  void intsecLine( Eigen::Vector<T,3>& ax, Eigen::Vector<T,3>& bx, Eigen::Vector<T,3>& x ) {
    // x = t * cx + ax
    Eigen::Vector<T,3> cx( bx - ax );
    //Eigen::Vector<T,3> n( a_, b_, c_ );
    Eigen::Vector<T,3> n = normal();
    T t = -1.0*( n.dot(ax) + d_ ) / n.dot(cx);
    //std::cout << "t = " << t << std::endl;
    x = t * cx + ax;
  };

  Eigen::Vector<T,3> normal() const { return Eigen::Vector<T,3>( a_, b_, c_ ); };

  friend std::ostream& operator <<( std::ostream& o, const Plane& p ) {
    return o << "a " << p.a_ << " b " << p.b_ << " c " << p.c_ << " d "
             << p.d_;
  };

private:

  T a_;
  T b_;
  T c_;
  T d_;

};

using Planed = Plane<double>;
using Planef = Plane<float>;

inline bool isLeftSide2d( const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const Eigen::Vector2d& v )
{
  if ( ((v2.x() - v1.x()) * (v.y()  - v1.y()) -
        (v.x()  - v1.x()) * (v2.y() - v1.y())) > 0.0 )
    return true;
  else
    return false;
}

inline bool isLineSegmentCrossing2d( Eigen::Vector2d& v1_s, Eigen::Vector2d& v1_e,
                                     Eigen::Vector2d& v2_s, Eigen::Vector2d& v2_e )
{
  bool s1 = isLeftSide2d( v2_s, v2_e, v1_s );
  bool e1 = isLeftSide2d( v2_s, v2_e, v1_e );
//    display("\t--1 %d %d\n", s1, e1 );
  if ( s1 == e1 ) return false;
  bool s2 = isLeftSide2d( v1_s, v1_e, v2_s );
  bool e2 = isLeftSide2d( v1_s, v1_e, v2_e );
//    display("\t--2 %d %d\n", s2, e2 );
  if ( s2 == e2 ) return false;
  return true;
}

inline double triArea2d(  const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c )
{
  return std::fabs((b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())) / 2.0;
}

//
// p が線分 ab の中に入っているかどうかの判定
//  from Graphics Gems I, Paeth, Alan W., "A Fast 2D Point-on-line Test"
//   p. 49-50, code: p. 654-655. A simplified version.
//
inline bool isPointOnLine2d( const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b )
{
  if ( std::fabs((b.y()-a.y())*(p.x()-a.x())-(p.y()-a.y())*(b.x()-a.x())) >=
       ((std::max)(std::fabs(b.x()-a.x()), std::fabs(b.y()-a.y()))) ) return false;
  if (((b.x()<a.x())&&(a.x()<p.x())) || ((b.y()<a.y())&&(a.y()<p.y()))) return false;
  if (((p.x()<a.x())&&(a.x()<b.x())) || ((p.y()<a.y())&&(a.y()<b.y()))) return false;
  if (((a.x()<b.x())&&(b.x()<p.x())) || ((a.y()<b.y())&&(b.y()<p.y()))) return false;
  if (((p.x()<b.x())&&(b.x()<a.x())) || ((p.y()<b.y())&&(b.y()<a.y()))) return false;

  return true;
}

// cosine between two vector3ds
inline double V3CosineBetweenVectors(Eigen::Vector3d& a, Eigen::Vector3d& b) {
  return a.dot(b) / (a.norm()*b.norm());
}

// angle between two vector3ds
inline double V3AngleBetweenVectors(Eigen::Vector3d& a, Eigen::Vector3d& b) {
  return std::fabs(std::atan2(a.cross(b).norm(), a.dot(b)));
  // double x = a.dot(b);
  // double y = a.cross(b).norm();
  // double ang = std::atan2(y, x);
  // if ( x < 0.0 ) ang += M_PI;
  // return ang;
}

// angle between two vector3ds a, b in the plane n
inline double V3AngleBetweenVectors(Eigen::Vector3d& a, Eigen::Vector3d& b, Eigen::Vector3d& n) {
#if 1
  double x = a.dot(b);
  double y = n.dot(a.cross(b));
  double ang = std::atan2(y, x);
  if (ang < 0.0) ang += DEG360;
  // if ( x < 0.0 ) ang += DEG360;
  return ang;
#endif
  // return std::fabs(std::atan2(a.cross(b).norm(), a.dot(b)));
}

// tan(θ/2) (θ:p0) for mean value coordinates
inline double tan2w( Eigen::Vector3d& a, Eigen::Vector3d& b, Eigen::Vector3d& c )
{
  Eigen::Vector3d ab(a-b);
  Eigen::Vector3d cb(c-b);
  double angle = V3AngleBetweenVectors(ab, cb);

  // std::cout << "\tcot " << 1.0 / std::tan( angle ) << " tan " << std::tan( M_PI_2 - angle ) << std::endl;
  // if ( 1.0 / std::tan( angle ) - std::tan( M_PI_2 - angle ) > 1.0e-05 )
  //   std::cout << "diff!" << std::endl;
  return std::tan( angle / 2.0 );
}

// set co-tangent of <abc
inline double cotAngle( Eigen::Vector3d& a, Eigen::Vector3d& b, Eigen::Vector3d& c )
{
  // gamma
  Eigen::Vector3d ab(a-b);
  Eigen::Vector3d cb(c-b);
  double ang = V3AngleBetweenVectors(ab, cb);
  // Vector3d ab(a-b);
  // Vector3d cb(c-b);
  // double ang = cb.angle(ab);
  return std::cos(ang)/std::sin(ang);
}

inline bool isPointInFace2d( const Eigen::Vector2d& p, const Eigen::Vector2d& v1,
                             const Eigen::Vector2d& v2, const Eigen::Vector2d& v3 )
{
  if ( (isLeftSide2d( v1, v2, p ) == true) &&
       (isLeftSide2d( v2, v3, p ) == true) &&
       (isLeftSide2d( v3, v1, p ) == true) )
    {
      return true;
    }
  else
    {
      return false;
    }
}

inline void findBarycentricCoordinate( const Eigen::Vector2d& p, const Eigen::Vector2d& v1,
                                       const Eigen::Vector2d& v2, const Eigen::Vector2d& v3,
                                       Eigen::Vector3d& bc )
{
  double area = triArea2d( v1, v2, v3 );
  //cout << area << endl;
  bc << (triArea2d( p, v2, v3 ) / area), (triArea2d( p, v3, v1 ) / area)
    , (triArea2d( p, v1, v2 ) / area);
  // bc.set( triArea2d( p, v2, v3 ) / area,
  //         triArea2d( p, v3, v1 ) / area,
  //         triArea2d( p, v1, v2 ) / area );
}

template <typename T>
inline void calcNormal( Eigen::Vector<T,3>& a, Eigen::Vector<T,3>& b, Eigen::Vector<T,3>& c, Eigen::Vector<T,3>& n )
{
  Eigen::Vector<T,3> v1( b - a );
  Eigen::Vector<T,3> v2( c - a );
  n = v1.cross( v2 );
  n.normalize();
}

template <typename T>
inline T calcArea( Eigen::Vector<T,3>& a, Eigen::Vector<T,3>& b, Eigen::Vector<T,3>& c )
{
  Eigen::Vector<T,3> v1( b - a );
  Eigen::Vector<T,3> v2( c - a );
  Eigen::Vector<T,3> t = v1.cross(v2);
  return t.norm() * .5;
}

template <typename T>
inline void calcArea( Eigen::Vector<T,3>& a, Eigen::Vector<T,3>& b, Eigen::Vector<T,3>& c, double& area )
{
  Eigen::Vector<T,3> v1( b - a );
  Eigen::Vector<T,3> v2( c - a );
  Eigen::Vector<T,3> t = v1.cross(v2);
  area = t.norm() * .5;
}

inline bool zero( double );
inline bool up( double );
inline bool down( double );
inline void bcp( std::vector<Eigen::Vector3d>&, std::vector<double>&, Eigen::Vector3d& );

//
// point to face distance
//
inline double triPointDistance( Eigen::Vector3d& p, std::vector<Eigen::Vector3d>& triverts,
                                std::vector<double>& bCoords )
{
//    double v0v2[3], v0v1[3], v1v2[3], pv1[3], pv2[3];
//    double vec[3];
//    double XX, ZZ, YY;
//    double WW, AA, BB;
//    double a, b, c;
//    double den;
//    double dis;
//    SUB( v0v2, triverts[0], triverts[2] );
//    SUB( v0v1, triverts[0], triverts[1] );
//    SUB( v1v2, triverts[1], triverts[2] );
//    SUB( pv1, p, triverts[1] );
//    SUB( pv2, p, triverts[2] );
//    /* 垂線の足を求める */
  
//    XX = DOT( v0v2, v0v1 ); YY = DOT( v1v2, v0v1 );
//    ZZ = DOT( v0v2, v1v2 ); WW = DOT( v1v2, v1v2 );
//    AA = DOT( pv2, v0v1 );  BB = DOT( pv2, v1v2 );
  
//    den = XX * WW - ZZ * YY;
//    if ( fabs(den) < 1.0e-10 ) return -1.0;
//    a = (WW * AA - YY * BB) / den;
//    b = (XX * BB - ZZ * AA) / den;
//    c = 1.0f - a - b;
//    BCP( vec, triverts, a, b, c );

  Eigen::Vector3d v0v2( triverts[0] - triverts[2] );
  Eigen::Vector3d v0v1( triverts[0] - triverts[1] );
  Eigen::Vector3d v1v2( triverts[1] - triverts[2] );
  Eigen::Vector3d pv1( p - triverts[1] );
  Eigen::Vector3d pv2( p - triverts[2] );
  
  double xx = v0v2.dot( v0v1 );
  double yy = v1v2.dot( v0v1 );
  double zz = v0v2.dot( v1v2 );
  double ww = v1v2.dot( v1v2 );
  double aa = pv2.dot( v0v1 );
  double bb = pv2.dot( v1v2 );

  double den = xx * ww - zz * yy;
  if ( std::fabs(den) < 1.0e-10 ) return -1.0;

  bCoords[0] = ( ww * aa - yy * bb ) / den;
  bCoords[1] = ( xx * bb - zz * aa ) / den;
  bCoords[2] = 1.0 - bCoords[0] - bCoords[1];
  Eigen::Vector3d q;
  bcp( triverts, bCoords, q );
  
  double dis = -1.0;
  if ( up(bCoords[0]) && up(bCoords[1]) && up(bCoords[2]) )
    { // (1)
      dis = Eigen::Vector3d(q-p).norm();
      // dis = q.distance( p );
//      dis = Distance2Points( vec, p );
    }
  else if ( zero(bCoords[0]) && up(bCoords[1]) && up(bCoords[2]) )
    { // (2)
      bCoords[0] = .0;
      bCoords[2] = 1.0 - bCoords[1];
      bcp( triverts, bCoords, q );
      dis = Eigen::Vector3d(q-p).norm();
      // dis = q.distance( p );
//        BCP( vec, triverts, a, b, c );
//        dis = Distance2Points( vec, p );
    }
  else if ( up(bCoords[0]) && zero(bCoords[1]) && up(bCoords[2]) )
    { // (3)
      bCoords[1] = .0;
      bCoords[0] = 1.0 - bCoords[2];
      bcp( triverts, bCoords, q );
      dis = Eigen::Vector3d(q-p).norm();
      //dis = q.distance( p );
//        b = 0.0f;
//        a = 1.0f - c;
//        BCP( vec, triverts, a, b, c );
//        dis = Distance2Points( vec, p );
    }
  else if ( up(bCoords[0]) && up(bCoords[1]) && zero(bCoords[2]) )
    { // (4)
      bCoords[2] = .0;
      bCoords[1] = 1.0 - bCoords[0];
      bcp( triverts, bCoords, q );
      dis = Eigen::Vector3d(q-p).norm();
      //dis = q.distance( p );
//      c = 0.0;
//      b = 1.0 - a;
//      BCP( vec, triverts, a, b, c );
//      dis = Distance2Points( vec, p );
    }
  else if ( down(bCoords[0]) && down(bCoords[1]) && up(bCoords[2]) )
    { // (8)
      // dis = triverts[2].distance( p );
      dis = Eigen::Vector3d(triverts[2]-p).norm();
//        dis = Distance2Points( triverts[2], p );
    }
  else if ( up(bCoords[0]) && down(bCoords[1]) && down(bCoords[2]) )
    { // (9)
      dis = Eigen::Vector3d(triverts[0]-p).norm();
      // dis = triverts[0].distance( p );
//        dis = Distance2Points( triverts[0], p );
    }
  else if ( down(bCoords[0]) && up(bCoords[1]) && down(bCoords[2]) )
    { // (10)
      dis = Eigen::Vector3d(triverts[1]-p).norm();
      // dis = triverts[1].distance( p );
//      dis = Distance2Points( triverts[1], p );
    }
  else if ( down(bCoords[0]) && up(bCoords[1]) && up(bCoords[2]) )
    { // (5)
      bCoords[0] = .0;
      bCoords[1] = bb / ww;
      bCoords[2] = 1.0 - bCoords[1];
//        a = 0.0f;
//        b = BB / WW;
//        c = 1.0f - b;
      if ( down(bCoords[2]) )
        {
//   dis = Distance2Points( triverts[1], p );
          dis = Eigen::Vector3d(triverts[1]-p).norm();
          // dis = triverts[1].distance( p );
        }
      else if ( down(bCoords[1]) )
        {
          dis = Eigen::Vector3d(triverts[2]-p).norm();
          // dis = triverts[2].distance( p );
//    dis = Distance2Points( triverts[2], p );
        }
      else
        {
          bcp( triverts, bCoords, q );
          dis = Eigen::Vector3d(q-p).norm();
          // dis = q.distance( p );
//    BCP( vec, triverts, a, b, c );
//    dis = Distance2Points( vec, p );
        }
    }
  else if ( up(bCoords[0]) && down(bCoords[1]) && up(bCoords[2]) )
    { // (6)
      bCoords[1] = .0;
      bCoords[0] = v0v2.dot( v0v2 ) / pv2.dot( v0v2 );
      bCoords[2] = 1.0 - bCoords[0];
//        b = 0.0;
//        a = DOT( v0v2, v0v2 ) / DOT( pv2, v0v2 );
//        c = 1.0 - a;
      if ( down(bCoords[2]) )
        {
          dis = Eigen::Vector3d(triverts[0]-p).norm();
          // dis = triverts[0].distance( p );
//    dis = Distance2Points( triverts[0], p );
        }
      else if ( down(bCoords[0]) )
        {
          dis = Eigen::Vector3d(triverts[2]-p).norm();
          // dis = triverts[2].distance( p );
//    dis = Distance2Points( triverts[2], p );
        }
      else
        {
          bcp( triverts, bCoords, q );
          dis = Eigen::Vector3d(q-p).norm();
          // dis = q.distance( p );
//    BCP( vec, triverts, a, b, c );
//    dis = Distance2Points( vec, p );
        }
    }
  else if ( up(bCoords[0]) && up(bCoords[1]) && down(bCoords[2]) )
    { // (7)
      bCoords[2] = .0;
      bCoords[0] = v0v1.dot( v0v1 ) / pv1.dot( v0v1 );
      bCoords[1] = 1.0 - bCoords[0];
//      c = 0.0;
//      a = DOT( v0v1, v0v1 ) / DOT( pv1, v0v1 );
//      b = 1.0 - a;
      if ( down(bCoords[0]) )
        {
          dis = Eigen::Vector3d(triverts[1]-p).norm();
          // dis = triverts[1].distance( p );
//    dis = Distance2Points( triverts[1], p );
        }
      else if ( down(bCoords[1]) )
        {
          dis = Eigen::Vector3d(triverts[0]-p).norm();
          // dis = triverts[0].distance( p );
//    dis = Distance2Points( triverts[0], p );
        }
      else
        {
          bcp( triverts, bCoords, q );
          dis = Eigen::Vector3d(q-p).norm();
          // dis = q.distance( p );
//    BCP( vec, triverts, a, b, c );
//    dis = Distance2Points( vec, p );
        }
    }

  return dis;
}

inline void V3Interpolate( Eigen::Vector3d& v1, Eigen::Vector3d& v2,
                           Eigen::Vector3d& v, double alpha )
{
  v = (1-alpha)*v1 + alpha*v2;
}

inline bool V3EpsilonEquals( Eigen::Vector3d& v1, Eigen::Vector3d& v2, double epsilon )
{
  return ((std::fabs(v1.x() - v2.x()) < epsilon) &&
          (std::fabs(v1.y() - v2.y()) < epsilon) &&
          (std::fabs(v1.z() - v2.z()) < epsilon));
}

inline double V3Triple( Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3 )
{
  double r = ( p1.x() * p2.y() * p3.z() + p1.y() * p2.z() * p3.x()
               + p1.z() * p2.x() * p3.y() - p1.z() * p2.y() * p3.x()
               - p1.x() * p2.z() * p3.y() - p1.y() * p2.x() * p3.z() );
  return r;
}

inline bool IntsecLinesParam3D( Eigen::Vector3d& v1_s, Eigen::Vector3d& v1_e,
                                Eigen::Vector3d& v2_s, Eigen::Vector3d& v2_e,
                                double* t, double* s )
{
  Eigen::Vector3d v1( v1_e - v1_s );
  Eigen::Vector3d v2( v2_e - v2_s );
  Eigen::Vector3d vc = v1.cross( v2 );
  
  // double den = vc.lengthSquared();
  double den = vc.squaredNorm();
#if 0
  if ( den < 1.0e-12 )
    {
      *t = .0f; *s = .0f;
      return false;
    }
#endif

  Eigen::Vector3d v( v2_s - v1_s );
  *t = V3Triple( v, v2, vc ) / den;
  *s = V3Triple( v, v1, vc ) / den;
  return true;
}

inline void bcp( std::vector<Eigen::Vector3d>& triverts,
                 std::vector<double>& bCoords, Eigen::Vector3d& p )
{
  // Eigen::Vector3d p0( triverts[0] ); p0.scale( bCoords[0] );
  // Eigen::Vector3d p1( triverts[1] ); p1.scale( bCoords[1] );
  // Eigen::Vector3d p2( triverts[2] ); p2.scale( bCoords[2] );
  // p.set( p0 + p1 + p2 );
  p = bCoords[0] * triverts[0] + bCoords[1] * triverts[1] + bCoords[2] * triverts[2];
}

inline bool zero( double a )
{
  if ( std::fabs(a) < ZEROEPS ) return true; else return false;
}

inline bool up( double a )
{
  if ( a > 0.0 ) return true; else return false;
}

inline bool down( double a )
{
  if ( a < 0.0 ) return true; else return false;
}

inline void V3Slerp( Eigen::Vector3d& v1, Eigen::Vector3d& v2, double t, Eigen::Vector3d& v )
{
  // double theta = v1.angle( v2 );
  double theta = V3AngleBetweenVectors( v1, v2 );
  double sin_c = std::sin( theta );
  double sin_a = std::sin( theta * (1.0 - t) );
  double sin_b = std::sin( theta * t );
  // Eigen::Vector3d tmp_a( v1 ); tmp_a.scale( sin_a / sin_c );
  // Eigen::Vector3d tmp_b( v2 ); tmp_b.scale( sin_b / sin_c );
  // v.set( tmp_a + tmp_b );
  v = (sin_a/sin_c)*v1 + (sin_b/sin_c)*v2;
}

inline void V3Slerp( Eigen::Vector3d& v1, Eigen::Vector3d& v2, double t, Eigen::Vector3d& n, Eigen::Vector3d& v )
{
  // double theta = v1.angle( v2 );
  double theta = V3AngleBetweenVectors( v1, v2, n );
  double sin_c = std::sin( theta );
  double sin_a = std::sin( theta * (1.0 - t) );
  double sin_b = std::sin( theta * t );
  // Eigen::Vector3d tmp_a( v1 ); tmp_a.scale( sin_a / sin_c );
  // Eigen::Vector3d tmp_b( v2 ); tmp_b.scale( sin_b / sin_c );
  // v.set( tmp_a + tmp_b );
  v = (sin_a/sin_c)*v1 + (sin_b/sin_c)*v2;
}

inline void V3SlerpF( Eigen::Vector3d& v1, Eigen::Vector3d& v2, double t, Eigen::Vector3d& v, bool flag )
{
  // double theta = v1.angle( v2 );
  double theta = V3AngleBetweenVectors( v1, v2 );
  if ( flag ) theta = 2.0*M_PI - theta;
  double sin_c = std::sin( theta );
  double sin_a = std::sin( theta * (1.0 - t) );
  double sin_b = std::sin( theta * t );
  // Eigen::Vector3d tmp_a( v1 ); tmp_a.scale( sin_a / sin_c );
  // Eigen::Vector3d tmp_b( v2 ); tmp_b.scale( sin_b / sin_c );
  // v.set( tmp_a + tmp_b );
  v = (sin_a/sin_c)*v1 + (sin_b/sin_c)*v2;
}

inline void convertCartesianToPolarCoord( Eigen::Vector3d& v, Eigen::Vector2d& p )
{
  double theta = std::acos( v.z() );
  double phi = std::acos( v.y() / std::sin(theta) );
  if ( (v.x() / std::sin(theta) ) < .0 ) phi += M_PI;
  // p.set( theta / M_PI, phi / (2.0f * M_PI) );
  p << (theta / M_PI), (phi / (2.0 * M_PI));
}

inline bool isIntersectBoxSphere( Eigen::Vector3d& minp, Eigen::Vector3d& maxp, Eigen::Vector3d& cen, double r )
{
  double dmin = .0;
  if( cen.x() < minp.x() ) dmin += ((cen.x() - minp.x()) * (cen.x() - minp.x()));
  if( cen.y() < minp.y() ) dmin += ((cen.y() - minp.y()) * (cen.y() - minp.y()));
  if( cen.z() < minp.z() ) dmin += ((cen.z() - minp.z()) * (cen.z() - minp.z()));
  if( cen.x() > maxp.x() ) dmin += ((cen.x() - maxp.x()) * (cen.x() - maxp.x()));
  if( cen.y() > maxp.y() ) dmin += ((cen.y() - maxp.y()) * (cen.y() - maxp.y()));
  if( cen.z() > maxp.z() ) dmin += ((cen.z() - maxp.z()) * (cen.z() - maxp.z()));
  double r2 = r*r;
  if( dmin <= r2 ) return true;
  return false;
}

inline void print_vec( Eigen::Vector3d& p ) {
  std::cout << p.x() << " " << p.y() << " " << p.z() << std::endl;
}

#endif // _VMPROC_HXX


