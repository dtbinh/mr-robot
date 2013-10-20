#ifndef MAPPING_H_INCLUDED
#define MAPPING_H_INCLUDED

#include <Eigen/Core>
#include <tf/transform_listener.h>

// Flags for the data contained in the matrix
// Unknow & Traversable = Traversable
// All the rest & NonTraversable = NonTraversable
enum CellState
{
	NonTraversable = 0,
	Traversable = 1,
	Unknown = 3	// Default state
};

class Cell{
public:
	double x;
	double y;
	Eigen::Vector3f normalVector;
	CellState state;
	double thetaThreshold;

	Cell();
	~Cell();

	void updateState();
};

#endif


