//#include "MpmEngine.h"
////#include <Eigen/Sparse>
//
////typedef Eigen::SparseMatrix<double> SpMat;
//
//void mpm::MpmEngine::MpmTimeStepSemiImplicitDirectSolve(real dt)
//{
//	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
//	GridNode* data = static_cast<GridNode*>(ptr);
//
//
//	// construct sparse matrix from data?
//	//SpMat M(GRID_SIZE_X * GRID_SIZE_Y, GRID_SIZE_X * GRID_SIZE_Y);
//	for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
//		GridNode gn = data[i];
//		
//		if (gn.m > 0.0) {
//			
//		}
//	}
//	glUnmapNamedBuffer(gridSSBO);
//}