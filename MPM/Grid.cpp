#include "Grid.h"
double mpm::GridNode::ComputeKE()
{
    return 0.5 * m * glm::dot(v, v);
}

double mpm::Grid::ComputeKE()
{
	KE = 0.0;
	for (int i = 0; i < GRID_SIZE_X; i++) {
		for (int j = 0; j < GRID_SIZE_Y; j++) {
			KE += nodes[i + j * GRID_SIZE_Y].ComputeKE();
		}
	}
	return KE;
}

