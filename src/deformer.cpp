#include "deformer.h"
#include <iostream>

using namespace std;
using namespace Eigen;
Deformer::Deformer() : mMesh(nullptr),
                       mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}


void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/

	Eigen::SparseMatrix< double > systemMat;

	/*====== Programming Assignment 2 ======*/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html
  vector<Triplet<double>> VList;

  VList.reserve((mMesh->vertices()).size() + (mMesh->boundaryEdges()).size() + (mMesh->edges().size() + mRoiList.size()));
  //copied from mesh.cpp to get matrix L
  for (Vertex* v : mMesh->vertices())
  {
    VList.push_back(Triplet<double>(v->index(), v->index(),-1));
    vector<Triplet<double>> VListwn;
    double w = 0;
    Vector3f RingAver(0,0,0);
    OneRingVertex vring(v);
    Vertex* vcur = new Vertex();
    vcur = nullptr; //same as uniform
    if (v->valence() > 2)
    {
      double w = 0;
      while (vcur = vring.nextVertex())
      {
        Vertex* valpha = new Vertex();
        valpha = nullptr;
        Vertex* vbeta = new Vertex();
        vbeta = nullptr;
        HEdge* hecur = new HEdge();
        OneRingHEdge hring(v); // Renew Every Time
        while (hecur = hring.nextHEdge())
        {
          if (hecur->end() == vcur)
          {
            valpha = hecur->twin()->next()->end();
            vbeta = hecur->next()->end();
          }
        }
        if (valpha == nullptr or vbeta == nullptr)
        {cout << "not found! " << endl;} //for debugging
        double cota = triangleCot(v->position(), valpha->position(), vcur->position());
        double cotb = triangleCot(v->position(), vbeta->position(), vcur->position());
        double wn = cota + cotb;

        VListwn.push_back(Triplet<double>(v->index(), vcur->index(),wn));
        w = w + wn;
        RingAver = RingAver + wn * vcur->position();
      }
      for (int j = 0; j < VListwn.size(); j++)
      {
        Triplet<double> vtxinfo = VListwn[j];
        VList.push_back(Triplet<double>(vtxinfo.row(), vtxinfo.col(), vtxinfo.value()/w));
      }
    }
  }

  for (int cp = 0; cp < mRoiList.size(); cp++)
  {
    VList.push_back(Triplet<double>(cp + (mMesh->vertices()).size(), mRoiList[cp]->index(), 1));
  }

  A.resize((mMesh->vertices()).size() + mRoiList.size(), (mMesh->vertices()).size());
  A.setFromTriplets(VList.begin(), VList.end());

  AT = A.transpose();
  systemMat = AT * A;
  /**/

	// Do factorization
	if (systemMat.nonZeros() > 0)
  {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		} else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
  // get inertial values
  VectorXd x0((mMesh->vertices()).size()), y0((mMesh->vertices()).size()), z0((mMesh->vertices()).size());
  // original value of x,y,z
  for (int m = 0; m < (mMesh->vertices().size()); m++)
  {
    x0(m) = mMesh->vertices()[m]->position()[0];
    y0(m) = mMesh->vertices()[m]->position()[1];
    z0(m) = mMesh->vertices()[m]->position()[2];
  }
  //original value of b
  int size_of_b = (mMesh->vertices()).size() + mRoiList.size();
  bx0.resize(size_of_b);
  by0.resize(size_of_b);
  bz0.resize(size_of_b);
  bx0 = A * x0;
  by0 = A * y0;
  bz0 = A * z0;
}

void Deformer::deform()
{
	if (mCholeskySolver == nullptr) {
		return;
	}
  //size of b
  int size_of_b = (mMesh->vertices()).size() + mRoiList.size();
  VectorXd bx(size_of_b), by(size_of_b), bz(size_of_b); // that after you drag some part
  // build b, first part is still delta, unchanged
  for (int k = 0; k < (mMesh->vertices()).size(); k++)
  {
    bx(k) = bx0(k);
    by(k) = by0(k);
    bz(k) = bz0(k);
  }
  // another half of b, that is moved position of handle
  for (int n = (mMesh->vertices()).size(); n < size_of_b; n++)
  {
    bx(n) = mRoiList[n - (mMesh->vertices()).size()]->position()[0];
    by(n) = mRoiList[n - (mMesh->vertices()).size()]->position()[1];
    bz(n) = mRoiList[n - (mMesh->vertices()).size()]->position()[2];
  }

  VectorXd ATbx(size_of_b), ATby(size_of_b), ATbz(size_of_b); // that
  ATbx = AT * bx;
  ATby = AT * by;
  ATbz = AT * bz;
  // Solve it
  VectorXd finalx, finaly, finalz;
  finalx = mCholeskySolver->solve(ATbx);
  finaly = mCholeskySolver->solve(ATby);
  finalz = mCholeskySolver->solve(ATbz);
  cout << "x are " << endl << finalx << endl << "y are " << endl << finaly << endl << "z are " << endl << finalz << endl;
  // give the value back to MESH
  for (int n = 0; n < (mMesh->vertices()).size(); n++)
  {
    mMesh->vertices()[n]->setPosition(Vector3f((float)finalx[n], (float)finaly[n], (float)finalz[n]));
  }

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques
	/* take place.
	/* Solve for the new vertex positions after the
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html


	/*====== Programming Assignment 2 ======*/
}
