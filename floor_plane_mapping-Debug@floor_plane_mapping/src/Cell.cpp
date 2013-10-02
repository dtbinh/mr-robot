/*
 * Cell class for mapping
 */

#include "Cell.h"

Cell::Cell(){

};

Cell::~Cell(){

}

/**
 * Transform the points
 */
void Cell::transformPoints(std::string from, std::string to){
	tf::TransformListener listener(ros::Duration(10));
	std::vector<geometry_msgs::PointStamped>::iterator pointsIterator;
	for (pointsIterator = this->points.begin();
			pointsIterator != this->points.end(); pointsIterator++) {
		// TODO Inplace replacement of the transformed point
		transformPoint(*pointsIterator, *pointsIterator, from, to);
	}
}

/**
 * Transform a single point
 */
void transformPoint(geometry_msgs::PointStamped& sourcePoint,
						   geometry_msgs::PointStamped& targetPoint,
						   std::string from,
						   std::string to) {
	tf::TransformListener listener(ros::Duration(10));
	sourcePoint.header.frame_id = from;
	sourcePoint.header.stamp = ros::Time();
	try {
		listener.waitForTransform(to, from,
				sourcePoint.header.stamp, ros::Duration(3.0));
		listener.transformPoint(to, sourcePoint, targetPoint);
	} catch (tf::TransformException& ex) {
		ROS_ERROR(
				"Received an exception trying to transform a point from \"%s\" to \"%s\": %s",
				from.c_str(),
				to.c_str(),
				ex.what());
	}
}


