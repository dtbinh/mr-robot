/*
 * Cell class for mapping
 */

#include "Cell.h"

Cell::Cell():x(0),y(0),normalVector(){

};

Cell::~Cell(){

}

void Cell::updateState(){
	Eigen::Vector3f zVector;
	double angle;
	zVector << 0, 0, 1;
	// Get the angle between the normalVector and the Z axis
	angle = acos(
			normalVector.dot(zVector) / (normalVector.norm() * zVector.norm()));
	// Set the threshold
	if (fabs(angle) <= thetaThreshold){
		state = Traversable;
	}
	else{
		state = NonTraversable;
//		ROS_INFO("ANGLE %f Coord %f %f",angle, x, y);
	}
}


