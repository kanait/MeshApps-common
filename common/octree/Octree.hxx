////////////////////////////////////////////////////////////////////
//
// $Id: Octree.hxx 2025/07/14 21:12:25 kanai Exp 
//
// Copyright (c) 2023-2025 by Takashi Kanai. All rights reserved.
//
////////////////////////////////////////////////////////////////////

#ifndef _OCTREE_HXX
#define _OCTREE_HXX 1

#include <vector>
#include <limits>
#include <memory>
#include <array>
#include <cmath>
//using namespace std;
#include "envDep.h"
#include "myEigen.hxx"
#include "FaceL.hxx"

#define  MAX_LEVEL 5

#include "tribox3.h"
#include "raytri.h"

// Octree のノードのクラス
class Octree : public std::enable_shared_from_this<Octree> {

public:

  Octree(){ init(); };
  Octree( Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax ) { init(); setBB( bbmin, bbmax ); };

  // parent_ と child_[8] ノードの初期化
  void init() {
    parent_ = nullptr;
    for ( int i = 0; i < 8; ++i ) child_[i] = nullptr;
    level_ = 0;
  };

  // Bounding Box (bbmin, bbmax) への値のセット
  void setBB( Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax ) {
    bbmin_ = bbmin;
    bbmax_ = bbmax;
  };

  void setParent( std::shared_ptr<Octree> parent ) { parent_ = parent; };
  void setChild( int id, std::shared_ptr<Octree> child ) { child_[id] = child; };
  void setLevel( int l ) { level_ = l; };

  int level() const { return level_; };
  Eigen::Vector3d& getBBmin() { return bbmin_; };
  Eigen::Vector3d& getBBmax() { return bbmax_; };

  // ノードの面リストへの面データの追加
  void addFaceList( std::shared_ptr<FaceL> fc ) {
    flist_.push_back( fc );
  };

  std::shared_ptr<Octree> child( int id ) { return child_[id]; };

  // Child ノードの生成
  std::shared_ptr<Octree> addChild( int id ) {
    // 範囲の計算
    Eigen::Vector3d bbmin, bbmax;
    calcChildRange( id, bbmin, bbmax );

    auto child = std::make_shared<Octree>( bbmin, bbmax );
    child->setLevel( level_ + 1 );

    child_[id] = child;

    // 親の設定を遅延させる（shared_from_this()が安全に呼び出せるようになってから）
    return child;
  };

  // child ノードの Bounding Box の計算
  void calcChildRange( int id, Eigen::Vector3d& cbbmin, Eigen::Vector3d& cbbmax ) {
    if( id < 4 ) {
      cbbmin.z() = bbmin_.z();
      cbbmax.z() = (bbmin_.z() + bbmax_.z()) / 2.0;

      switch( id ) {
      case 0:
        cbbmin.x() = bbmin_.x();
        cbbmin.y() = bbmin_.y();
        cbbmax.x() = (bbmax_.x() + bbmin_.x()) / 2.0;
        cbbmax.y() = (bbmax_.y() + bbmin_.y()) / 2.0;
        break;

      case 1:
        cbbmin.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmin.y() = bbmin_.y();
        cbbmax.x() = bbmax_.x();
        cbbmax.y() = (bbmin_.y() + bbmax_.y())/ 2.0;    
        break;

      case 2:
        cbbmin.x() = bbmin_.x();
        cbbmin.y() = (bbmin_.y() + bbmax_.y())/ 2.0;
        cbbmax.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmax.y() = bbmax_.y();
        break;

      case 3:
        cbbmin.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmin.y() = (bbmin_.y() + bbmax_.y())/ 2.0;
        cbbmax.x() = bbmax_.x();
        cbbmax.y() = bbmax_.y();
        break;
      }
    } else {
      cbbmin.z() = (bbmin_.z() + bbmax_.z()) / 2.0;
      cbbmax.z() = bbmax_.z();

      switch( id ) {
      case 4:
        cbbmin.x() = bbmin_.x();
        cbbmin.y() = bbmin_.y();
        cbbmax.x() = (bbmax_.x() + bbmin_.x()) / 2.0;
        cbbmax.y() = (bbmax_.y() + bbmin_.y()) / 2.0;
        break;

      case 5:
        cbbmin.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmin.y() = bbmin_.y();
        cbbmax.x() = bbmax_.x();
        cbbmax.y() = (bbmin_.y() + bbmax_.y())/ 2.0;    
        break;

      case 6:
        cbbmin.x() = bbmin_.x();
        cbbmin.y() = (bbmin_.y() + bbmax_.y())/ 2.0;
        cbbmax.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmax.y() = bbmax_.y();
        break;

      case 7:
        cbbmin.x() = (bbmin_.x() + bbmax_.x())/ 2.0;
        cbbmin.y() = (bbmin_.y() + bbmax_.y())/ 2.0;
        cbbmax.x() = bbmax_.x();
        cbbmax.y() = bbmax_.y();
        break;
      }
    }
  };

  //
  // node->child[i] の中に fc が入っているかどうか調べ
  // 入っていれば node->child[i] を再帰的に調べる．
  // あるレベルに到達したときに face list に加える．
  //
  void addFaceToOctree( std::shared_ptr<FaceL> fc ) {
    if ( level_ == MAX_LEVEL ) {
      addFaceList( fc );
      return;
    }

    for ( int i = 0; i < 8; ++i ) {
      // 面 fc が 子供の範囲内に入っているかどうかをチェック
      Eigen::Vector3d bbmin, bbmax;
      calcChildRange( i, bbmin, bbmax );
      if ( isFaceOveralapBox( fc, bbmin, bbmax ) ) { // 入っている場合
        // child[i] がなければ作成
        if (child_[i] == nullptr) child_[i] = addChild(i);
        try {
          child_[i]->setParent( shared_from_this() );
        } catch (const std::bad_weak_ptr&) {
          // shared_from_this()が失敗した場合は親を設定しない
          // これは通常の動作に影響しない
        }
        child_[i]->addFaceToOctree( fc );
      }
    }
  };

  //
  // 面がボックスに少しでも入っているかどうかをチェック
  //
  bool isFaceOveralapBox( std::shared_ptr<FaceL> face, Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax ) {

    if (!face) return false;

    float boxcenter[3], boxhalfsize[3];
    boxcenter[0] = (float) ( bbmax.x() + bbmin.x() ) / 2.0f;
    boxcenter[1] = (float) ( bbmax.y() + bbmin.y() ) / 2.0f;
    boxcenter[2] = (float) ( bbmax.z() + bbmin.z() ) / 2.0f;
    boxhalfsize[0] = (float) ( bbmax.x() - bbmin.x() ) / 2.0f;
    boxhalfsize[1] = (float) ( bbmax.y() - bbmin.y() ) / 2.0f;
    boxhalfsize[2] = (float) ( bbmax.z() - bbmin.z() ) / 2.0f;

    // bool isFaceInRange( Face *face, Point3d *bbmin, Point3d *bbmax ) {
    float triverts[3][3];
    int i = 0;
    for (auto& he : face->halfedges()) {
      Eigen::Vector3d& p = he->vertex()->point();
      triverts[i][0] = (float) p.x();
      triverts[i][1] = (float) p.y();
      triverts[i][2] = (float) p.z();
      ++i;
    }

    return triBoxOverlap( boxcenter, boxhalfsize, triverts );
  };

  // 直線とボックスの交差判定
  // http://marupeke296.com/COL_3D_No18_LineAndAABB.html
  bool isRayIntersect( Eigen::Vector3d& pos, Eigen::Vector3d& dir ) {

    double t_max = std::numeric_limits<double>::max();   // AABB からレイが外に出る時刻
    double t_min = -std::numeric_limits<double>::max();  // AABB にレイが侵入する時刻

    for (int i=0; i<3; i++) {
      double t1, t2;
      if ( i==0 ) {
        if (std::abs(dir.x()) < 1e-10) {
          // レイがx軸に平行な場合
          if (pos.x() < bbmin_.x() || pos.x() > bbmax_.x()) {
            return false;
          }
          continue;
        }
        t1 = (bbmin_.x() - pos.x())/dir.x();
        t2 = (bbmax_.x() - pos.x())/dir.x();
      } else if (i == 1) {
        if (std::abs(dir.y()) < 1e-10) {
          // レイがy軸に平行な場合
          if (pos.y() < bbmin_.y() || pos.y() > bbmax_.y()) {
            return false;
          }
          continue;
        }
        t1 = (bbmin_.y() - pos.y())/dir.y();
        t2 = (bbmax_.y() - pos.y())/dir.y();
      } else if (i == 2) {
        if (std::abs(dir.z()) < 1e-10) {
          // レイがz軸に平行な場合
          if (pos.z() < bbmin_.z() || pos.z() > bbmax_.z()) {
            return false;
          }
          continue;
        }
        t1 = (bbmin_.z() - pos.z())/dir.z();
        t2 = (bbmax_.z() - pos.z())/dir.z();
      }
      double t_near = std::min(t1, t2);
      double t_far = std::max(t1, t2);
      t_max = std::min(t_max, t_far);
      t_min = std::max(t_min, t_near);

      //
      // レイが外に出る時刻と侵入する時刻が逆転している => 交差していない
      //
      if (t_min > t_max) return false;
    }

    return true;
  };

  // ray と flist_ に入っている面との交差判定
  // 複数入っている場合は，pos に一番近い点を出力する
  std::shared_ptr<FaceL> intersectRayFaces( Eigen::Vector3d& pos, Eigen::Vector3d& dir,
                            Eigen::Vector3d& near_p ) {

    std::shared_ptr<FaceL> near_fc = nullptr;
    for (int i = 0; i < flist_.size(); ++i ) {
      std::shared_ptr<FaceL> fc = flist_[i];
      if (!fc) continue; // 安全チェック
      
      double orig[3], ddir[3], vert0[3], vert1[3], vert2[3];
      orig[0] = pos.x(); orig[1] = pos.y(); orig[2] = pos.z();
      ddir[0] = dir.x(); ddir[1] = dir.y(); ddir[2] = dir.z();
      int j = 0;
      Eigen::Vector3d p0, p1, p2;
      for ( auto& he : fc->halfedges() ) {
        if (j == 0) {
          p0 = he->vertex()->point();
          vert0[0] = p0.x();
          vert0[1] = p0.y();
          vert0[2] = p0.z();
        } else if (j == 1) {
          p1 = he->vertex()->point();
          vert1[0] = p1.x();
          vert1[1] = p1.y();
          vert1[2] = p1.z();
        } else if (j == 2) {
          p2 = he->vertex()->point();
          vert2[0] = p2.x();
          vert2[1] = p2.y();
          vert2[2] = p2.z();
        }
        ++j;
      }
      double t,u,v;
      if ( intersect_triangle2(orig, ddir, vert0, vert1, vert2,
                               &t, &u, &v) ) {
        Eigen::Vector3d p = (1.0-u-v) * p0 + u * p1 + v * p2;
        if ( near_fc != nullptr ) {
          if ( (pos - near_p).norm() > (pos - p).norm() ) {
            near_fc = fc;
            near_p = p;
          }
        } else {
          near_fc = fc;
          near_p = p;
        }
      }
    }

    return near_fc;
  };

private:

  int level_;
  Eigen::Vector3d bbmin_, bbmax_;
  std::shared_ptr<Octree> parent_;
  std::array<std::shared_ptr<Octree>, 8> child_;
  std::vector<std::shared_ptr<FaceL> > flist_;
};

#endif // _OCTREE_HXX
