#pragma once

#include "Constants.h"
#include "PointCloud.h"
#include "Grid.h"

namespace mpm {

	namespace Basis {


		enum class BasisType {
			LINEAR = 0,
			QUADRATIC_B_SPLINE = 1,
			CUBIC_B_SPLINE = 2
		};


		struct NodeGetter {

			NodeGetter() {}

			NodeGetter(mpm::MaterialPoint& mp, std::shared_ptr<mpm::Grid> grid, BasisType bType) {
				this->grid = grid;

				basisType = bType;

				switch (basisType) {
				case BasisType::LINEAR:
					botLeftNodeIndex = ivec2(std::floor(mp.x.x), std::floor(mp.x.y));
					maxRelativeIndex = ivec2(1, 1);
					break;

				case BasisType::QUADRATIC_B_SPLINE:
					botLeftNodeIndex = ivec2(std::floor(mp.x.x) - 1, std::floor(mp.x.y) - 1);
					maxRelativeIndex = ivec2(3, 3);
					break;

				case BasisType::CUBIC_B_SPLINE:
					botLeftNodeIndex = ivec2(std::floor(mp.x.x) - 1, std::floor(mp.x.y) - 1);
					maxRelativeIndex = ivec2(3, 3);
					break;
				default:
					break;
				}
				

			}

			bool Complete() {
				return complete;
			}

			bool IsNodeOK() {
				return nodeOk;
			}



			GridNode& NextNode() { // Note: always check if complete before calling this

				//std::cout << i << ", " << j << std::endl;

				if (InBounds(botLeftNodeIndex.x + i, botLeftNodeIndex.y + j, GRID_SIZE_X, GRID_SIZE_Y)) {
					size_t index = size_t(botLeftNodeIndex.x + i) + size_t(botLeftNodeIndex.y + j) * size_t(GRID_SIZE_Y);
					//std::cout << "[" << grid->nodes[index].x.x << ", " << grid->nodes[index].x.y << "]" << std::endl;
					nodeOk = true;
					NodeIndexIncrement();
					return grid->nodes[index];
				}
				else {
					nodeOk = false;
					NodeIndexIncrement();
					return grid->nodes[0];
				}

				

			}

			double ShapeFunction(double x) {
				switch(basisType) {
				case BasisType::LINEAR:
					return LinearShape(x);
					break;
				case BasisType::QUADRATIC_B_SPLINE:
					return QuadraticBSpline(x);
					break;
				case BasisType::CUBIC_B_SPLINE:
					return CubicBSpline(x);
					break;
				default:
					return 0;
				}
			}

			double ShapeFunctionSlope(double x) {
				switch (basisType) {
				case BasisType::LINEAR:
					return LinearShapeSlope(x);
					break;
				case BasisType::QUADRATIC_B_SPLINE:
					return QuadraticBSplineSlope(x);
					break;
				case BasisType::CUBIC_B_SPLINE:
					return CubicBSplineSlope(x);
					break;
				default:
					return 0;
				}
			}


		private:
			bool complete = false;
			bool nodeOk;
			int i = 0;
			int j = 0;
			ivec2 botLeftNodeIndex;
			ivec2 maxRelativeIndex;
			std::shared_ptr<Grid> grid;


			BasisType basisType = BasisType::CUBIC_B_SPLINE;

			void NodeIndexIncrement() {
				j++;
				if (j > maxRelativeIndex.y) {
					if (i == maxRelativeIndex.x) {
						complete = true;
					}
					else {
						j = 0;
						i++;
					}
				}
			}
		};
	}
}