#ifndef DEFORMER_H
#define DEFORMER_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "mesh.h"
using namespace Eigen;
// Deform mesh using Laplacian coordinates
class Deformer {
public:
	Deformer();
	~Deformer();

	void setMesh(Mesh* mesh);

	/*====== Programming Assignment 2 ======*/
	// This is the place where the editing techniques take place
	void deform();
	/*====== Programming Assignment 2 ======*/

private:
	/*====== Programming Assignment 2 ======*/
	// Build left hand side matrix and pre-factorize it
	void buildSystemMat();
	/*====== Programming Assignment 2 ======*/
	SparseMatrix<double> A; // Ax = b where x are positions
	SparseMatrix<double> AT;
	VectorXd bx0, by0, bz0; // initial value of b
	void clear();

	Mesh* mMesh;
	std::vector< Vertex* > mRoiList;
	// Solver for pre-factorizing the system matrix of the deformation
	Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >* mCholeskySolver;
};

#endif // DEFORMER_H
