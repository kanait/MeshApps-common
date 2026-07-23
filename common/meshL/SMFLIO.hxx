////////////////////////////////////////////////////////////////////
//
// $Id: SMFLIO.hxx 2025/07/17 23:55:44 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _SMFLIO_HXX
#define _SMFLIO_HXX

#include <fstream>
#include <iostream>
#include <vector>

#include "envDep.h"
//using namespace std;

#include "LIO.hxx"
#include "myEigen.hxx"
#include "strutil.h"
#include "tokenizer.h"

class SMFLIO : public LIO {
 public:
  SMFLIO() : LIO(), isSaveNormalization_(false){};
  SMFLIO(MeshL& mesh) : LIO(mesh), isSaveNormalization_(false){};
  ~SMFLIO(){};

  void setSaveNormalization(bool f) { isSaveNormalization_ = f; };
  bool isSaveNormalization() const { return isSaveNormalization_; };

  bool inputFromFile(const char* const filename) {
    //
    // file open
    //
    std::ifstream ifs;
    ifs.open(filename);
    if (!ifs.is_open()) {
      std::cerr << "Cannot open " << filename << std::endl;
      return false;
    }

    // for refering vertex pointer
    std::vector<std::shared_ptr<VertexL> > vertex_p;
    std::vector<std::shared_ptr<NormalL> > normal_p;
    std::vector<std::shared_ptr<TexcoordL> > tcoord_p;
    int v_count = 0;
    int f_count = 0;
    //
    // parse smf
    //
    std::string cline;
    StrUtil strutil;
    while (getline(ifs, cline, '\n')) {
      std::string fw;
      strutil.first_word(cline, fw);

      // comment
      if (mycomment(cline[0])) continue;

      
      // read vertices
      else if (!fw.compare("v")) {
        std::istringstream isstr(cline);

        // "v"
        std::string str;
        isstr >> str;
        double x, y, z;
        isstr >> x;
        isstr >> y;
        isstr >> z;
        Eigen::Vector3d p(x, y, z);

        std::shared_ptr<VertexL> vt = mesh().addVertex(p);
        double r, g, b;
        if (isstr >> r) {
          isstr >> g >> b;
          vt->setColor(r, g, b);
        }
        vertex_p.push_back(vt);

        ++v_count;
      }

      // read normals (Wavefront OBJ: vn; legacy SMF-style: n)
      else if (!fw.compare("vn") || !fw.compare("n")) {
        std::istringstream isstr(cline);

        std::string str;
        isstr >> str;
        double x, y, z;
        isstr >> x;
        isstr >> y;
        isstr >> z;
        Eigen::Vector3d p(x, y, z);

        std::shared_ptr<NormalL> nm = mesh().addNormal(p);
        normal_p.push_back(nm);
      }

      // read texture coordinates
      else if (!fw.compare("r") || !fw.compare("vt")) {
        std::istringstream isstr(cline);

        // "r"
        std::string str;
        isstr >> str;
        double x, y, z;
        isstr >> x;
        isstr >> y;
        if (isstr)
          isstr >> z;
        else
          z = .0;
        Eigen::Vector3d p(x, y, z);

        std::shared_ptr<TexcoordL> tc = mesh().addTexcoord(p);
        tcoord_p.push_back(tc);
      }

      // read faces (Wavefront: f v, f v/vt, f v//vn, f v/vt/vn)
      else if (!fw.compare("f")) {
        std::istringstream isstr(cline);

        // "f"
        std::string str;
        isstr >> str;

        std::shared_ptr<FaceL> fc = mesh().addFace();
        while (isstr >> str) {
          // Keep empty fields so "1//2" → {"1","","2"}
          std::vector<std::string> parts;
          {
            size_t start = 0;
            for (size_t i = 0; i <= str.size(); ++i) {
              if (i == str.size() || str[i] == '/') {
                parts.push_back(str.substr(start, i - start));
                start = i + 1;
              }
            }
          }
          if (parts.empty() || parts[0].empty()) continue;

          int id;
          {
            std::istringstream ai(parts[0]);
            ai >> id;
          }
          if (id < 1 || id > static_cast<int>(vertex_p.size())) continue;
          std::shared_ptr<HalfedgeL> he =
              mesh().addHalfedge(fc, vertex_p[static_cast<size_t>(id - 1)]);

          auto parseIndex = [](const std::string& s, int& out) -> bool {
            if (s.empty()) return false;
            std::istringstream ai(s);
            return static_cast<bool>(ai >> out);
          };

          if (parts.size() == 2) {
            // f v/vt  or legacy f v/n (when only normals exist)
            int idx = 0;
            if (parseIndex(parts[1], idx)) {
              if (!(mesh().texcoords().empty())) {
                if (idx >= 1 && idx <= static_cast<int>(tcoord_p.size()))
                  he->setTexcoord(tcoord_p[static_cast<size_t>(idx - 1)]);
              } else if (!(mesh().normals().empty())) {
                if (idx >= 1 && idx <= static_cast<int>(normal_p.size()))
                  he->setNormal(normal_p[static_cast<size_t>(idx - 1)]);
              }
            }
          } else if (parts.size() >= 3) {
            // f v/vt/vn or f v//vn
            int idx = 0;
            if (parseIndex(parts[1], idx) && !(mesh().texcoords().empty())) {
              if (idx >= 1 && idx <= static_cast<int>(tcoord_p.size()))
                he->setTexcoord(tcoord_p[static_cast<size_t>(idx - 1)]);
            }
            if (parseIndex(parts[2], idx) && !(mesh().normals().empty())) {
              if (idx >= 1 && idx <= static_cast<int>(normal_p.size()))
                he->setNormal(normal_p[static_cast<size_t>(idx - 1)]);
            }
          }
        }

        fc->calcNormal();

        ++f_count;
      }

      // read boundary loop
      else if (!fw.compare("b")) {
        std::istringstream isstr(cline);

        // "b"
        std::string str;
        isstr >> str;

        std::shared_ptr<BLoopL> bl = mesh().addBLoop();
        int id;
        while (isstr >> id) {
          if (id > 0) {
            bl->addIsCorner(true);
          } else {
            bl->addIsCorner(false);
            id *= -1;  // reverse
          }
          bl->addVertex(vertex_p[id - 1]);
        }
        std::cout << "b: " << bl->vertices().size() << " vertices."
                  << std::endl;
      }

    }  // while

    ifs.close();

    mesh().printInfo();

    return true;
  };

  bool outputToFile(const char* const filename, bool isSaveNormal,
                    bool isSaveTexcoord, bool isSaveBLoop) {
    setSaveNormal(isSaveNormal);
    setSaveTexcoord(isSaveTexcoord);
    setSaveBLoop(isSaveBLoop);
    return outputToFile(filename);
  };

  bool outputToFile(const char* const filename, bool isSaveNormal,
                    bool isSaveTexcoord, bool isSaveBLoop, bool isSaveColor) {
    setSaveNormal(isSaveNormal);
    setSaveTexcoord(isSaveTexcoord);
    setSaveBLoop(isSaveBLoop);
    setSaveColor(isSaveColor);
    return outputToFile(filename);
  };

  bool outputToFile(const char* const filename, bool isSaveNormalization) {
    setSaveNormalization(isSaveNormalization);
    return outputToFile(filename);
  };

  // bool outputToFile( const char* const );
  bool outputToFile(const char* const filename) {
    mesh().printInfo();

    std::ofstream ofs(filename);
    if (!ofs) return false;

    // header
    int vn = mesh().vertices().size();
    int tn = mesh().texcoords().size();
    int nn = mesh().normals().size();
    int fn = mesh().faces().size();

    ofs << "####" << std::endl;
    ofs << "#" << std::endl;
    ofs << "# OBJ File Generated by hsphparam" << std::endl;
    ofs << "#" << std::endl;
    ofs << "####" << std::endl;
    ofs << "#" << std::endl;
    ofs << "# Vertices: " << vn << std::endl;
    if (nn && isSaveNormal()) ofs << "# Normals: " << nn << std::endl;
    if (tn && isSaveTexcoord()) ofs << "# Texcoords: " << tn << std::endl;
    ofs << "# Faces: " << fn << std::endl;
    ofs << "#" << std::endl;
    ofs << "####" << std::endl;

    if (vn) {
      int id = 1;
      for ( auto& vt : mesh().vertices() ) {
        Eigen::Vector3d& p = vt->point();
        if (mesh().isNormalized() && !(isSaveNormalization())) {
          p *= mesh().maxLength();
          p += mesh().center();
        }
        ofs << "v " << p.x() << " " << p.y() << " " << p.z();
        if (isSaveColor() && vt->hasColor()) {
          const Eigen::Vector3d& c = vt->color();
          ofs << " " << c.x() << " " << c.y() << " " << c.z();
        }
        ofs << std::endl;
        vt->setID(id);
        id++;
      }
    }

    // int nn = mesh().normals().size();
    if (nn && isSaveNormal()) {
      int id = 1;
      for ( auto& nm : mesh().normals() ) {
        Eigen::Vector3d& p = nm->point();
        ofs << "vn " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        nm->setID(id);
        id++;
      }
    }

    // int tn = mesh().texcoords().size();
    if (tn && isSaveTexcoord()) {
      int id = 1;
      for ( auto& tc : mesh().texcoords() ) {
        Eigen::Vector3d& p = tc->point();
        ofs << "vt " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        tc->setID(id);
        id++;
      }
    }

    if (fn) {
      for ( auto& fc : mesh().faces() ) {
        ofs << "f ";
        for ( auto& he : fc->halfedges() ) {
          ofs << he->vertex()->id();
          const bool has_n = he->normal() && isSaveNormal();
          const bool has_t = he->texcoord() && isSaveTexcoord();
          // Wavefront: f v, f v/vt, f v//vn, f v/vt/vn
          if (has_t || has_n) {
            ofs << "/";
            if (has_t) ofs << he->texcoord()->id();
            if (has_n) ofs << "/" << he->normal()->id();
          }
          ofs << " ";
        }
        ofs << std::endl;
      }
    }

    // save bloop
    if (isSaveBLoop() && mesh().bloops().size()) {
      std::shared_ptr<BLoopL> bl = *(mesh().bloops().begin());

      ofs << "b\t";
      for (unsigned int i = 0; i < bl->vertices().size(); ++i) {
        int id = bl->vertex(i)->id();

        if (!(bl->isCorner(i))) id *= -1;  // reverse

        ofs << id << " ";
      }
      ofs << std::endl;
    }

    ofs.close();

    // reset numbers

    if (vn) {
      int id = 0;
      for ( auto& vt : mesh().vertices() ) {
        vt->setID(id);
        id++;
      }
    }
    if (nn && isSaveNormal()) {
      int id = 0;
      for ( auto& nm : mesh().normals() ) {
        nm->setID(id);
        id++;
      }
    }
    if (tn && isSaveTexcoord()) {
      int id = 1;
      for ( auto& tc : mesh().texcoords() ) {
        tc->setID(id);
        id++;
      }
    }

    return true;
  };

 private:
  bool isSaveNormalization_;
};
#endif  // _SMFLIO_H
