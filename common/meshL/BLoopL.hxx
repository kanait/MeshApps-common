////////////////////////////////////////////////////////////////////
//
// $Id: BLoopL.hxx 2025/07/17 14:35:40 kanai Exp 
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _BLOOPL_HXX
#define _BLOOPL_HXX

#include "envDep.h"

#include <cassert>
#include <vector>
#include <list>
#include <memory>
//using namespace std;

#include "LoopL.hxx"
#include "MeshUtiL.hxx"

class BLoopL : public LoopL {

public:

  BLoopL() : LoopL() { init(); };
  BLoopL( int id ) : LoopL(id) { init(); };
  ~BLoopL() {
    vertices().clear();
    halfedges().clear();
    isCorner_.clear();
  };

  void init() {
    LoopL::init();
    setIsClosedLoop( true );
  };

  // iter
  void setBIter(std::list<std::shared_ptr<BLoopL> >::iterator iter) { iterb_ = iter; }
  std::list<std::shared_ptr<BLoopL> >::iterator biter() const { return iterb_; }

  std::shared_ptr<HalfedgeL> findHalfedgeBloop( std::shared_ptr<VertexL> o, std::shared_ptr<VertexL> vt ) {
    VertexLCirculator vc( o );
    std::shared_ptr<HalfedgeL> he = vc.beginHalfedgeL();
    do {
      if ( he->next()->vertex() == vt ) return he;

      std::shared_ptr<HalfedgeL> mate = he->mate();
      if ( mate )
      {
        if ( mate->vertex() == vt ) return mate;
      }
      he = vc.nextHalfedgeL();
    } while ( (he != nullptr) && (he != vc.firstHalfedgeL()) );

    return nullptr;
  };


  // create halfedges from vertices
  //   input : vertices:  v0, v1, v2, ....., vn
  //   output: halfedges: h0(v1, v0), h1(v2, v1), .... hn( v0, vn )
  //
  void createHalfedges() {
    if ( !(vertices().size()) ) return;

    for ( unsigned int i = 0; i < vertices().size(); ++i )
      {
        std::shared_ptr<VertexL> ovt = vertex(i);
        std::shared_ptr<VertexL> nvt = ( i == (vertices().size()-1) ) ?vertex(0): vertex(i+1);
        std::shared_ptr<HalfedgeL> he = findHalfedgeBloop( ovt, nvt );
        assert( he );
        assert( he->isBoundary() );
        addHalfedge( he );
      }
    //   reset();
  }

  // Corner flag
  // uses in reading a file
  void addIsCorner( bool f ) {
    isCorner_.push_back( f );
  };
  bool isCorner( int i ) const {
    assert( i < (int) isCorner_.size() );
    return isCorner_[i];
  };
  void setCorner( int i, bool f ) {
    assert( i < (int) isCorner_.size() );
    isCorner_[i] = f;
  };


  int numCornerVertices() {
    int c = 0;
    for ( int i = 0; i < (int)vertices().size(); ++i )
      {
        if ( isCorner(i) ) ++c;
      }
    return c;
  };

  bool isVertex( std::shared_ptr<VertexL> vt ) {
    for ( int i = 0; i < (int)vertices().size(); ++i )
      {
        if ( vertex(i) == vt ) return true;
      }
    return false;
  };

  // void optimize( int );
  void optimize( int n_corner ) {
    std::vector<double> arr_length;
    double sum_length = .0;
    int i;
    for ( i = 0; i < (int)vertices().size(); ++i )
      {
        Eigen::Vector3d& p0 = vertex(i)->point();
        Eigen::Vector3d& p1 = ( i != (int) vertices().size()-1 )
          ? vertex(i+1)->point() : vertex(0)->point();
        double length = Eigen::Vector3d(p0-p1).norm();
        arr_length.push_back( length );
        sum_length += length;
      }

    double length4 = sum_length / (double) n_corner;

    sum_length = .0;
    int n_new_corner = 1;
    for ( i = 0; i < (int)vertices().size(); ++i )
      {
        setCorner( i, false );

        if ( sum_length + arr_length[i] > length4 )
          {
            if ( n_new_corner < n_corner )
              {
                sum_length = .0;
                setCorner( i, true );
                n_new_corner++;
              }
          }
        else
          sum_length += arr_length[i];
      }

    setCorner( 0, true );
  };

  void print() {
    // vertices
    unsigned int i;
    for ( i = 0; i < vertices().size(); ++i )
      {
        std::cout << "vt " << vertices()[i]->id() << std::endl;
      }
    // halfedges
    for ( i = 0; i < halfedges().size(); ++i )
      {
        std::cout << "he " << halfedges()[i]->id() << " "
                  << "sv " << halfedges()[i]->vertex()->id() << " "
                  << "ev " << halfedges()[i]->next()->vertex()->id()
                  << std::endl;
      }

  };

private:

  // vertices:  v0, v1, v2, ....., vn
  // * v0 MUST be a corner vertex!
  
  // halfedges: h0(v1, v0), h1(v2, v1), .... hn(v0, vn)
  
  // // list iterator of MeshL
  std::list<std::shared_ptr<BLoopL> >::iterator iterb_;

  // isCorner
  std::vector<bool> isCorner_;

};

#endif // _BLOOPL_HXX

