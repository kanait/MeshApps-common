////////////////////////////////////////////////////////////////////
//
// $Id: VertexLCirculator.hxx 2025/07/21 18:05:54 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
// ----------------------------------------------------------------
// 使用例
// VertexLCirculator circulator(vertex);
// 
// // 参照で受け取る場合
// for (auto& neighbor_vertex : circulator.vertices()) {
//     // neighbor_vertex は std::shared_ptr<VertexL>&
//     std::cout << "Neighbor vertex ID: " << neighbor_vertex->id() << std::endl;
// }
// 
// // 値で受け取る場合（従来通りでも使用可能）
// for (auto neighbor_vertex : circulator.vertices()) {
//     // neighbor_vertex は std::shared_ptr<VertexL>
//     std::cout << "Neighbor vertex ID: " << neighbor_vertex->id() << std::endl;
// }
// 
// // 面とハーフエッジも同様
// for (auto& face : circulator.faces()) {
//     // face は std::shared_ptr<FaceL>&
// }
// 
// for (auto& halfedge : circulator.halfedges()) {
//     // halfedge は std::shared_ptr<HalfedgeL>&
// }
//
// // Generic navigation example (循環検出機能付き)
// VertexLCirculator circulator(vertex);
// 
// // 頂点循環（一周すると自動的に終了）
// circulator.beginVertices();
// do {
//     auto current = circulator.currentVertex();
//     if (current) {
//         std::cout << "Vertex ID: " << current->id() << std::endl;
//         // process current vertex
//     }
// } while (circulator.next());
//
// // 面循環（一周すると自動的に終了）
// circulator.beginFaces();
// do {
//     auto current = circulator.currentFace();
//     if (current) {
//         std::cout << "Face ID: " << current->id() << std::endl;
//         // process current face
//     }
// } while (circulator.next());
//
// // ハーフエッジ循環（一周すると自動的に終了）
// circulator.beginHalfedges();
// do {
//     auto current = circulator.currentHalfedge();
//     if (current) {
//         // process current halfedge
//         // circulator.prev() も使用可能（ハーフエッジのみ）
//     }
// } while (circulator.next());
//
////////////////////////////////////////////////////////////////////

#ifndef _VERTEXLCIRCULATOR_HXX
#define _VERTEXLCIRCULATOR_HXX

#include "VertexL.hxx"
#include "HalfedgeL.hxx"
#include "FaceL.hxx"
#include <memory>
#include <iterator>

// Forward declaration
class VertexLCirculator;

// Range class for vertices
class VertexRange {
private:
  std::shared_ptr<VertexL> center_vertex_;

public:
  class Iterator {
  private:
    VertexLCirculator* circulator_;
    std::shared_ptr<VertexL> current_;
    std::shared_ptr<VertexL> first_;
    bool is_end_;
    bool is_first_iteration_;

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = std::shared_ptr<VertexL>;
    using difference_type = std::ptrdiff_t;
    using pointer = std::shared_ptr<VertexL>*;
    using reference = std::shared_ptr<VertexL>&;

    Iterator(VertexLCirculator* circ, bool end = false);
    ~Iterator();
    Iterator(const Iterator& other);
    Iterator& operator=(const Iterator& other);
    Iterator& operator++();
    Iterator operator++(int);
    bool operator==(const Iterator& other) const;
    bool operator!=(const Iterator& other) const;
    std::shared_ptr<VertexL>& operator*() const;
    std::shared_ptr<VertexL>* operator->() const;
  };

  VertexRange(std::shared_ptr<VertexL> center) : center_vertex_(center) {}
  Iterator begin();
  Iterator end();
};

// Range class for faces
class FaceRange {
private:
  std::shared_ptr<VertexL> center_vertex_;

public:
  class Iterator {
  private:
    VertexLCirculator* circulator_;
    std::shared_ptr<FaceL> current_;
    std::shared_ptr<FaceL> first_;
    bool is_end_;
    bool is_first_iteration_;

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = std::shared_ptr<FaceL>;
    using difference_type = std::ptrdiff_t;
    using pointer = std::shared_ptr<FaceL>*;
    using reference = std::shared_ptr<FaceL>&;

    Iterator(VertexLCirculator* circ, bool end = false);
    ~Iterator();
    Iterator(const Iterator& other);
    Iterator& operator=(const Iterator& other);
    Iterator& operator++();
    Iterator operator++(int);
    bool operator==(const Iterator& other) const;
    bool operator!=(const Iterator& other) const;
    std::shared_ptr<FaceL>& operator*() const;
    std::shared_ptr<FaceL>* operator->() const;
  };

  FaceRange(std::shared_ptr<VertexL> center) : center_vertex_(center) {}
  Iterator begin();
  Iterator end();
};

// Range class for halfedges
class HalfedgeRange {
private:
  std::shared_ptr<VertexL> center_vertex_;

public:
  class Iterator {
  private:
    VertexLCirculator* circulator_;
    std::shared_ptr<HalfedgeL> current_;
    std::shared_ptr<HalfedgeL> first_;
    bool is_end_;
    bool is_first_iteration_;

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = std::shared_ptr<HalfedgeL>;
    using difference_type = std::ptrdiff_t;
    using pointer = std::shared_ptr<HalfedgeL>*;
    using reference = std::shared_ptr<HalfedgeL>&;

    Iterator(VertexLCirculator* circ, bool end = false);
    ~Iterator();
    Iterator(const Iterator& other);
    Iterator& operator=(const Iterator& other);
    Iterator& operator++();
    Iterator operator++(int);
    bool operator==(const Iterator& other) const;
    bool operator!=(const Iterator& other) const;
    std::shared_ptr<HalfedgeL>& operator*() const;
    std::shared_ptr<HalfedgeL>* operator->() const;
  };

  HalfedgeRange(std::shared_ptr<VertexL> center) : center_vertex_(center) {}
  Iterator begin();
  Iterator end();
};

class VertexLCirculator {

  std::shared_ptr<VertexL> vt_;

protected:

  std::shared_ptr<HalfedgeL> temp_halfedge_;

public:

  enum class CirculatorType {
    NONE,
    VERTEX,
    FACE, 
    HALFEDGE,
    REV_HALFEDGE
  };

protected:
  CirculatorType current_type_;
  std::shared_ptr<VertexL> current_vertex_;
  std::shared_ptr<FaceL> current_face_;
  std::shared_ptr<HalfedgeL> current_halfedge_;
  
  // For loop detection
  std::shared_ptr<VertexL> first_vertex_;
  std::shared_ptr<FaceL> first_face_;
  std::shared_ptr<HalfedgeL> first_halfedge_;
  bool has_started_;

public:

  VertexLCirculator() { clear(); };
  VertexLCirculator(std::shared_ptr<VertexL> vt) { clear(); setVertex(vt); }
  ~VertexLCirculator() {};

  void clear() { 
    vt_ = nullptr; 
    temp_halfedge_ = nullptr; 
    current_type_ = CirculatorType::NONE;
    current_vertex_ = nullptr;
    current_face_ = nullptr;
    current_halfedge_ = nullptr;
    first_vertex_ = nullptr;
    first_face_ = nullptr;
    first_halfedge_ = nullptr;
    has_started_ = false;
  };
  void setVertex(std::shared_ptr<VertexL> vt) { vt_ = vt; };

  // Range-based for loop support
  VertexRange vertices() { return VertexRange(vt_); };
  FaceRange faces() { return FaceRange(vt_); };
  HalfedgeRange halfedges() { return HalfedgeRange(vt_); };

  // Generic navigation methods
  void beginVertices() {
    current_type_ = CirculatorType::VERTEX;
    current_vertex_ = beginVertexL();
    first_vertex_ = firstVertexL();  // Store first vertex for loop detection
    has_started_ = false;
  };

  void beginFaces() {
    current_type_ = CirculatorType::FACE;
    current_face_ = beginFaceL();
    first_face_ = firstFaceL();  // Store first face for loop detection
    has_started_ = false;
  };

  void beginHalfedges() {
    current_type_ = CirculatorType::HALFEDGE;
    current_halfedge_ = beginHalfedgeL();
    first_halfedge_ = firstHalfedgeL();  // Store first halfedge for loop detection
    has_started_ = false;
  };

  void beginRevHalfedges() {
    current_type_ = CirculatorType::REV_HALFEDGE;
    current_halfedge_ = beginRevHalfedgeL();
    first_halfedge_ = firstRevHalfedgeL();  // Store first reverse halfedge for loop detection
    has_started_ = false;
  };

  bool next() {
    if (!has_started_) {
      has_started_ = true;
      return true; // First element is already set in begin methods
    }

    switch (current_type_) {
      case CirculatorType::VERTEX:
        current_vertex_ = nextVertexL();
        if (current_vertex_ == nullptr || current_vertex_ == first_vertex_) {
          return false; // End of circulation
        }
        return true;
      case CirculatorType::FACE:
        current_face_ = nextFaceL();
        if (current_face_ == nullptr || current_face_ == first_face_) {
          return false; // End of circulation
        }
        return true;
      case CirculatorType::HALFEDGE:
        current_halfedge_ = nextHalfedgeL();
        if (current_halfedge_ == nullptr || current_halfedge_ == first_halfedge_) {
          return false; // End of circulation
        }
        return true;
      case CirculatorType::REV_HALFEDGE:
        current_halfedge_ = nextRevHalfedgeL();
        if (current_halfedge_ == nullptr || current_halfedge_ == first_halfedge_) {
          return false; // End of circulation
        }
        return true;
      default:
        return false;
    }
  };

  bool prev() {
    switch (current_type_) {
      case CirculatorType::HALFEDGE:
        current_halfedge_ = prevHalfedgeL();
        return current_halfedge_ != nullptr;
      case CirculatorType::REV_HALFEDGE:
        // For reverse halfedge, prev is the reverse direction
        current_halfedge_ = nextRevHalfedgeL();
        return current_halfedge_ != nullptr;
      default:
        // Vertex and Face don't have direct prev methods in the original implementation
        return false;
    }
  };

  // Get current elements
  std::shared_ptr<VertexL> currentVertex() const {
    return (current_type_ == CirculatorType::VERTEX) ? current_vertex_ : nullptr;
  };

  std::shared_ptr<FaceL> currentFace() const {
    return (current_type_ == CirculatorType::FACE) ? current_face_ : nullptr;
  };

  std::shared_ptr<HalfedgeL> currentHalfedge() const {
    return (current_type_ == CirculatorType::HALFEDGE || current_type_ == CirculatorType::REV_HALFEDGE) ? 
           current_halfedge_ : nullptr;
  };

  CirculatorType getCurrentType() const {
    return current_type_;
  };

  //
  // vertex -> face
  //
  std::shared_ptr<FaceL> beginFaceL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_->face();
  };

  std::shared_ptr<FaceL> nextFaceL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    return temp_halfedge_->face();
  };

  std::shared_ptr<FaceL> firstFaceL() {
    return vt_->halfedge()->face();
  };

  std::shared_ptr<FaceL> lastFaceL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->face();
  };

  //
  // vertex -> vertex
  //
  std::shared_ptr<VertexL> beginVertexL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_->next()->vertex();
  };

  std::shared_ptr<VertexL> nextVertexL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    if ( he == nullptr ) return nullptr;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr )
      {
        // last vertex
        return he->prev()->vertex();
      }
    return temp_halfedge_->next()->vertex();
  };

  std::shared_ptr<VertexL> firstVertexL() {
    return vt_->halfedge()->next()->vertex();
  };

  std::shared_ptr<VertexL> lastVertexL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->next()->vertex();
  };

  //
  // vertex -> halfedge
  //
  std::shared_ptr<HalfedgeL> beginHalfedgeL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_;
  };

  std::shared_ptr<HalfedgeL> nextHalfedgeL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    return temp_halfedge_;
  };

  std::shared_ptr<HalfedgeL> prevHalfedgeL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->mate()->next();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    return temp_halfedge_;
  };

  std::shared_ptr<HalfedgeL> firstHalfedgeL() {
    return vt_->halfedge();
  };

  std::shared_ptr<HalfedgeL> lastHalfedgeL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->next();
  };

  //
  //  vertex -> reverse halfedge
  //
  std::shared_ptr<HalfedgeL> beginRevHalfedgeL() {
    assert( vt_->halfedge() );
    temp_halfedge_ = vt_->halfedge()->prev();
    return temp_halfedge_;
  };

  std::shared_ptr<HalfedgeL> nextRevHalfedgeL() {
    if ( temp_halfedge_->mate() == nullptr ) return nullptr;
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->mate()->prev();
    return temp_halfedge_;
  };

  std::shared_ptr<HalfedgeL> firstRevHalfedgeL() {
    return vt_->halfedge()->prev();
  };

  std::shared_ptr<HalfedgeL> lastRevHalfedgeL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate();
  };

  int num_vertices() {
    int count=0;
    std::shared_ptr<VertexL> vt = beginVertexL();
    if (vt == nullptr) return 0;
    std::shared_ptr<VertexL> first_vt = firstVertexL();
    do { 
      count++; 
      vt = nextVertexL();
    } while ( (vt != first_vt) && (vt != nullptr) );
    return count;
  };

  int num_faces() {
    int count=0;
    std::shared_ptr<FaceL> fc = beginFaceL();
    if (fc == nullptr) return 0;
    std::shared_ptr<FaceL> first_fc = firstFaceL();
    do { 
      count++; 
      fc = nextFaceL(); 
    } while ( (fc != first_fc) && (fc != nullptr) );
    return count;
  };

  void setfirstHalfedge(std::shared_ptr<HalfedgeL> he) {
    vt_->setHalfedge(he);
  };

  // Make ranges friend classes to access private members
  friend class VertexRange;
  friend class FaceRange;
  friend class HalfedgeRange;
};

// Implementation of VertexRange::Iterator
inline VertexRange::Iterator::Iterator(VertexLCirculator* circ, bool end) 
  : circulator_(nullptr), is_end_(end), is_first_iteration_(true) {
  if (!is_end_ && circ && circ->vt_) {
    // Create our own circulator instance to avoid state conflicts
    circulator_ = new VertexLCirculator(circ->vt_);
    current_ = circulator_->beginVertexL();
    first_ = circulator_->firstVertexL();
    if (!current_) {
      is_end_ = true;
    }
  } else {
    current_ = nullptr;
    first_ = nullptr;
    is_end_ = true;
  }
}

inline VertexRange::Iterator::~Iterator() {
  delete circulator_;
}

inline VertexRange::Iterator::Iterator(const Iterator& other) 
  : circulator_(nullptr), current_(other.current_), first_(other.first_), 
    is_end_(other.is_end_), is_first_iteration_(other.is_first_iteration_) {
  if (other.circulator_) {
    circulator_ = new VertexLCirculator(*other.circulator_);
  }
}

inline VertexRange::Iterator& VertexRange::Iterator::operator=(const Iterator& other) {
  if (this != &other) {
    delete circulator_;
    circulator_ = nullptr;
    if (other.circulator_) {
      circulator_ = new VertexLCirculator(*other.circulator_);
    }
    current_ = other.current_;
    first_ = other.first_;
    is_end_ = other.is_end_;
    is_first_iteration_ = other.is_first_iteration_;
  }
  return *this;
}

inline VertexRange::Iterator& VertexRange::Iterator::operator++() {
  if (is_end_) return *this;
  
  if (is_first_iteration_) {
    is_first_iteration_ = false;
  }
  
  current_ = circulator_->nextVertexL();
  
  if (!current_ || current_ == first_) {
    is_end_ = true;
  }
  
  return *this;
}

inline VertexRange::Iterator VertexRange::Iterator::operator++(int) {
  Iterator tmp = *this;
  ++(*this);
  return tmp;
}

inline bool VertexRange::Iterator::operator==(const Iterator& other) const {
  return is_end_ == other.is_end_ && (is_end_ || current_ == other.current_);
}

inline bool VertexRange::Iterator::operator!=(const Iterator& other) const {
  return !(*this == other);
}

inline std::shared_ptr<VertexL>& VertexRange::Iterator::operator*() const {
  return const_cast<std::shared_ptr<VertexL>&>(current_);
}

inline std::shared_ptr<VertexL>* VertexRange::Iterator::operator->() const {
  return const_cast<std::shared_ptr<VertexL>*>(&current_);
}

inline VertexRange::Iterator VertexRange::begin() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, false);
}

inline VertexRange::Iterator VertexRange::end() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, true);
}

// Implementation of FaceRange::Iterator
inline FaceRange::Iterator::Iterator(VertexLCirculator* circ, bool end) 
  : circulator_(nullptr), is_end_(end), is_first_iteration_(true) {
  if (!is_end_ && circ && circ->vt_) {
    // Create our own circulator instance to avoid state conflicts
    circulator_ = new VertexLCirculator(circ->vt_);
    current_ = circulator_->beginFaceL();
    first_ = circulator_->firstFaceL();
    if (!current_) {
      is_end_ = true;
    }
  } else {
    current_ = nullptr;
    first_ = nullptr;
    is_end_ = true;
  }
}

inline FaceRange::Iterator::~Iterator() {
  delete circulator_;
}

inline FaceRange::Iterator::Iterator(const Iterator& other) 
  : circulator_(nullptr), current_(other.current_), first_(other.first_), 
    is_end_(other.is_end_), is_first_iteration_(other.is_first_iteration_) {
  if (other.circulator_) {
    circulator_ = new VertexLCirculator(*other.circulator_);
  }
}

inline FaceRange::Iterator& FaceRange::Iterator::operator=(const Iterator& other) {
  if (this != &other) {
    delete circulator_;
    circulator_ = nullptr;
    if (other.circulator_) {
      circulator_ = new VertexLCirculator(*other.circulator_);
    }
    current_ = other.current_;
    first_ = other.first_;
    is_end_ = other.is_end_;
    is_first_iteration_ = other.is_first_iteration_;
  }
  return *this;
}

inline FaceRange::Iterator& FaceRange::Iterator::operator++() {
  if (is_end_) return *this;
  
  if (is_first_iteration_) {
    is_first_iteration_ = false;
  }
  
  current_ = circulator_->nextFaceL();
  
  if (!current_ || current_ == first_) {
    is_end_ = true;
  }
  
  return *this;
}

inline FaceRange::Iterator FaceRange::Iterator::operator++(int) {
  Iterator tmp = *this;
  ++(*this);
  return tmp;
}

inline bool FaceRange::Iterator::operator==(const Iterator& other) const {
  return is_end_ == other.is_end_ && (is_end_ || current_ == other.current_);
}

inline bool FaceRange::Iterator::operator!=(const Iterator& other) const {
  return !(*this == other);
}

inline std::shared_ptr<FaceL>& FaceRange::Iterator::operator*() const {
  return const_cast<std::shared_ptr<FaceL>&>(current_);
}

inline std::shared_ptr<FaceL>* FaceRange::Iterator::operator->() const {
  return const_cast<std::shared_ptr<FaceL>*>(&current_);
}

inline FaceRange::Iterator FaceRange::begin() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, false);
}

inline FaceRange::Iterator FaceRange::end() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, true);
}

// Implementation of HalfedgeRange::Iterator
inline HalfedgeRange::Iterator::Iterator(VertexLCirculator* circ, bool end) 
  : circulator_(nullptr), is_end_(end), is_first_iteration_(true) {
  if (!is_end_ && circ && circ->vt_) {
    // Create our own circulator instance to avoid state conflicts
    circulator_ = new VertexLCirculator(circ->vt_);
    current_ = circulator_->beginHalfedgeL();
    first_ = circulator_->firstHalfedgeL();
    if (!current_) {
      is_end_ = true;
    }
  } else {
    current_ = nullptr;
    first_ = nullptr;
    is_end_ = true;
  }
}

inline HalfedgeRange::Iterator::~Iterator() {
  delete circulator_;
}

inline HalfedgeRange::Iterator::Iterator(const Iterator& other) 
  : circulator_(nullptr), current_(other.current_), first_(other.first_), 
    is_end_(other.is_end_), is_first_iteration_(other.is_first_iteration_) {
  if (other.circulator_) {
    circulator_ = new VertexLCirculator(*other.circulator_);
  }
}

inline HalfedgeRange::Iterator& HalfedgeRange::Iterator::operator=(const Iterator& other) {
  if (this != &other) {
    delete circulator_;
    circulator_ = nullptr;
    if (other.circulator_) {
      circulator_ = new VertexLCirculator(*other.circulator_);
    }
    current_ = other.current_;
    first_ = other.first_;
    is_end_ = other.is_end_;
    is_first_iteration_ = other.is_first_iteration_;
  }
  return *this;
}

inline HalfedgeRange::Iterator& HalfedgeRange::Iterator::operator++() {
  if (is_end_) return *this;
  
  if (is_first_iteration_) {
    is_first_iteration_ = false;
  }
  
  current_ = circulator_->nextHalfedgeL();
  
  if (!current_ || current_ == first_) {
    is_end_ = true;
  }
  
  return *this;
}

inline HalfedgeRange::Iterator HalfedgeRange::Iterator::operator++(int) {
  Iterator tmp = *this;
  ++(*this);
  return tmp;
}

inline bool HalfedgeRange::Iterator::operator==(const Iterator& other) const {
  return is_end_ == other.is_end_ && (is_end_ || current_ == other.current_);
}

inline bool HalfedgeRange::Iterator::operator!=(const Iterator& other) const {
  return !(*this == other);
}

inline std::shared_ptr<HalfedgeL>& HalfedgeRange::Iterator::operator*() const {
  return const_cast<std::shared_ptr<HalfedgeL>&>(current_);
}

inline std::shared_ptr<HalfedgeL>* HalfedgeRange::Iterator::operator->() const {
  return const_cast<std::shared_ptr<HalfedgeL>*>(&current_);
}

inline HalfedgeRange::Iterator HalfedgeRange::begin() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, false);
}

inline HalfedgeRange::Iterator HalfedgeRange::end() {
  VertexLCirculator temp_circ(center_vertex_);
  return Iterator(&temp_circ, true);
}

#endif // _VERTEXLCIRCULATOR_HXX
