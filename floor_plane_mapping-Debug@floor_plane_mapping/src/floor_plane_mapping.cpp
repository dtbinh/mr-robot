#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <sys/time.h>
#include "Cartography.h"
#include "Cell.h"


class FloorPlaneMapping {
protected:
	ros::Subscriber scan_sub_;
	tf::TransformListener listener_;

	ros::NodeHandle nh_;
	std::string base_frame_;
	std::string world_frame_;

	double max_range_;
	double tolerance;
	double traverse_threshold; // Angle threshold to determine if traversable
	double normal_estimation_radius; // Normal estimation radius in meters

	pcl::PointCloud<pcl::PointXYZ> lastpc_;

	Cell* pCell;
	Cartography *pCartography;
protected:
	// ROS Callbacks

	void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg){
		/**
		 * Transformation of the point clouds
		 */
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		listener_.waitForTransform(base_frame_, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
				msg->header.frame_id, lastpc_, listener_);

		// cloudPtr -> point cloud in the world frame
		listener_.waitForTransform(world_frame_, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(world_frame_, msg->header.stamp, temp,
				msg->header.frame_id, *cloudPtr, listener_);

		/*
		 * Normal estimation for the point cloud
		 */
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(cloudPtr);

		// KD Tree search method
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		normalEstimation.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloudNormalsPtr(new pcl::PointCloud<pcl::Normal>);
		// Use all neighbors in a sphere of radius of "normal_estimation_radis" in meters
		normalEstimation.setRadiusSearch(normal_estimation_radius);
		// Compute the features
		normalEstimation.compute(*cloudNormalsPtr);

		/**
		 * Filter the points in the point cloud
		 */
		unsigned int n = temp.size();
		std::vector<size_t> pidx;
		// First count the useful points
		for (unsigned int i = 0; i < n; i++) {
			float x = temp[i].x;
			float y = temp[i].y;
			float d = hypot(x, y);
			if (d < 1e-2) {
				// Bogus point, ignore
				continue;
			}
			x = lastpc_[i].x;
			y = lastpc_[i].y;
			d = hypot(x, y);
			if (d > max_range_) {
				// too far, ignore
				continue;
			}
			pidx.push_back(i);
		}

		/*
		 * Update and mapping
		 */
		int pidx_size = pidx.size();
		for(int i = 0; i<pidx_size; i++){
			pCell->x = cloudPtr->points[pidx[i]].x;
			pCell->y = cloudPtr->points[pidx[i]].y;
			pCell->normalVector <<
					cloudNormalsPtr->points[pidx[i]].normal_x,
					cloudNormalsPtr->points[pidx[i]].normal_y,
					fabs(cloudNormalsPtr->points[pidx[i]].normal_z);
			// Update cell state
			pCell->updateState();
			// Mapping
			pCartography->Update(pCell->x,pCell->y,pCell->state);
		}
		// TODO
		// BUG L84 Cartography.cpp
		pCartography->PublishImage();
	}

public:
	FloorPlaneMapping() :
			nh_("~") {
		nh_.param("base_frame", base_frame_, std::string("/body"));
		nh_.param("world_frame", world_frame_, std::string("/world"));
		nh_.param("max_range", max_range_, 5.0);
		nh_.param("normal_estimation_radius",normal_estimation_radius,0.03);
		nh_.param("traverse_threshold",traverse_threshold, 1.2);

		ROS_INFO("Mapping");

		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneMapping::pc_Callback,
				this);

		pCartography = new Cartography(nh_, 1.0, 10);
		pCell = new Cell();
		pCell->thetaThreshold = traverse_threshold;
	}

	~FloorPlaneMapping(){
		delete pCell;
		delete pCartography;
	}

	ros::NodeHandle getNodeHanlder(){
		return nh_;
	}

	// Get the elapsed time from start to end
	long getElapsedTime(timeval& start, timeval& end) {
		long mtime, seconds, useconds;
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		return mtime;
	}

};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "floor_plane_mapping");
	FloorPlaneMapping fp;
	ros::spin();
	return 0;
}

