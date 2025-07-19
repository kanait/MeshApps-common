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
        vertex_p.push_back(vt);

        ++v_count;
      }

      // read normals
      else if (!fw.compare("n")) {
        std::istringstream isstr(cline);

        // "n"
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

      // read faces
      else if (!fw.compare("f")) {
        std::istringstream isstr(cline);

        // "f"
        std::string str;
        isstr >> str;

        std::shared_ptr<FaceL> fc = mesh().addFace();
        while (isstr >> str) {
          tokenizer tok(str, "/");

          // vertex
          std::string val = tok.next();

          int id;
          std::istringstream ai(val);
          ai >> id;
          std::shared_ptr<HalfedgeL> he = mesh().addHalfedge(fc, vertex_p[id - 1]);

          // normal
          if (!(mesh().normals().empty())) {
            val = tok.next();
            std::istringstream ai(val);
            ai >> id;
            he->setNormal(normal_p[id - 1]);
          }
          // texcoord
          if (!(mesh().texcoords().empty())) {
            val = tok.next();
            std::istringstream ai(val);
            ai >> id;
            he->setTexcoord(tcoord_p[id - 1]);
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
        ofs << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        vt->setID(id);
        id++;
      }
    }

    // int nn = mesh().normals().size();
    if (nn && isSaveNormal()) {
      int id = 1;
      for ( auto& nm : mesh().normals() ) {
        Eigen::Vector3d& p = nm->point();
        ofs << "n\t" << p.x() << " " << p.y() << " " << p.z() << std::endl;
        nm->setID(id);
        id++;
      }
    }

    // int tn = mesh().texcoords().size();
    if (tn && isSaveTexcoord()) {
      int id = 1;
      for ( auto& tc : mesh().texcoords() ) {
        Eigen::Vector3d& p = tc->point();
        ofs << "vt\t" << p.x() << " " << p.y() << " " << p.z() << std::endl;
        tc->setID(id);
        id++;
      }
    }

    if (fn) {
      for ( auto& fc : mesh().faces() ) {
        ofs << "f ";
        for ( auto& he : fc->halfedges() ) {
          ofs << he->vertex()->id();
          if (he->normal() && isSaveNormal())
            ofs << "/" << he->normal()->id();
          if (he->texcoord() && isSaveTexcoord())
            ofs << "/" << he->texcoord()->id();
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
