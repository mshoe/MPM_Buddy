#include "Grid.h"
double mpm::GridNode::ComputeKE()
{
    return 0.5 * m * glm::dot(v, v);
}

mpm::Grid::Grid(size_t grid_size_x, size_t grid_size_y)
{
	grid_dim_x = grid_size_x;
	grid_dim_y = grid_size_y;

	nodes = std::vector<GridNode>(grid_dim_x * grid_dim_y, GridNode());

	for (int i = 0; i < grid_dim_x; i++) {
		for (int j = 0; j < grid_dim_y; j++) {
			GetNode(i, j).x = vec2(double(i), double(j));
		}
	}

	/*for (int i = 0; i < grid_dim_x; i++) {
		for (int j = 0; j < grid_dim_y; j++) {
			std::cout << i << ", " << j << std::endl;
			std::cout << GetNode(i, j).x.x << ", " << GetNode(i, j).x.y << std::endl << std::endl;
		}
	}*/

	

}


mpm::GridNode& mpm::Grid::GetNode(size_t i, size_t j)
{
	
	return nodes[j * grid_dim_x + i];
}

double mpm::Grid::ComputeKE()
{
	KE = 0.0;
	for (int i = 0; i < grid_dim_x; i++) {
		for (int j = 0; j < grid_dim_y; j++) {
			KE += GetNode(i, j).ComputeKE();
		}
	}
	return KE;
}

