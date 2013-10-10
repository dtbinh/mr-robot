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
	std::vector<geometry_msgs::PointStamped&> points;
	Eigen::Vector3f& normalVector;
	CellState state;
	double angleThreshold;

	Cell();
	~Cell();

	// Transform the points
	void transformPoints(std::string from, std::string to);
	void updateState();
private:
	// Transform a single
	void transformPoint(geometry_msgs::PointStamped& sourcePoint,
			geometry_msgs::PointStamped& targetPoint,
			std::string from,
			std::string to);
};

#endif

