#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
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
	ros::Publisher marker_pub_;
	tf::TransformListener listener_;

	ros::NodeHandle nh_;
	std::string base_frame_;
	std::string world_frame_;

	double max_range_;
	double tolerance;
	double traverse_threshold;
	int n_samples;

	pcl::PointCloud<pcl::PointXYZ> lastpc_;

	// Mapping
	Cell* pCell;
	Cartography *pCartography;

protected:
	// ROS Callbacks

	void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		listener_.waitForTransform(base_frame_, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
				msg->header.frame_id, lastpc_, listener_);

		//
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

		//
		// BEGIN TODO
		// Finding planes: z = a*x + b*y + c
		// Remember to use the n_samples, tolerance
		n = pidx.size();
		size_t best = 0;
		double X[3] = { 0, 0, 0 };
		// Set the timer
		struct timeval start, end;
		gettimeofday(&start, NULL);
		//
		ROS_INFO("%d useful points out of %d", (int)n, (int)temp.size());


		// Set up a cell
		pCell = new Cell();
		std::vector<geometry_msgs::PointStamped> cellPoints;
		Eigen::Vector3f normalVector;

		// Let's ransack!
		for (unsigned int i = 0; i < (unsigned) n_samples; i++) {
			if (n == 0) {
				break;
			}
			Eigen::Vector3f samplePoints[3];
			// Initialize the random seed
			// srand(time(NULL));
			// Pick up 3 random points
			for (int j = 0; j < 3; j++) {
				int index = getRandomIndex((int) n);
				samplePoints[j] << lastpc_[pidx[index]].x, lastpc_[pidx[index]].y, lastpc_[pidx[index]].z;
			}
			// Calculate the plane ax+by+cz+d=0
			Eigen::Vector3f p = samplePoints[1] - samplePoints[0];
			Eigen::Vector3f q = samplePoints[2] - samplePoints[1];
			normalVector = p.cross(q);
			normalVector.normalize(); // Normalize the vector
			double d = -samplePoints[1].dot(normalVector);

			// Evaluation
			size_t score = 0;
			for (int i = 0; i < n; i++) {
				// Calculate the score for this model
				if (calcDistance(lastpc_[pidx[i]], normalVector, d)
						<= tolerance) {
					// Store the points to the cell
					cellPoints.push_back(CloudPointToPointStamped(lastpc_[pidx[i]]));
					score++;
				}
				// Update if a better model is found
				if (score > best) {
					best = score;
					X[0] = normalVector[0] / -normalVector[2];
					X[1] = normalVector[1] / -normalVector[2];
					X[2] = d / -normalVector[2];
				}
			}
		}

		pCell->normalVector = normalVector;
		pCell->points = cellPoints;
		updateMapping(*pCell);
		// At the end, make sure to store the best plane estimate in X
		// X = {a,b,c}. This will be used for display
		gettimeofday(&end, NULL); // Stop the timer
		ROS_INFO("Time elapsed %ld ms", getElapsedTime(start,end));
		ROS_INFO("Score %d", (int)best);
		// END OF TODO

/*		ROS_INFO(

				"Extracted floor plane: z = %.2fx + %.2fy + %.2f", X[0], X[1], X[2]);*/
		Eigen::Vector3f O, u, v, w;
		w << X[0], X[1], -1.0;
		w /= w.norm();
		O << 1.0, 0.0, 1.0 * X[0] + 0.0 * X[1] + X[2];
		u << 2.0, 0.0, 2.0 * X[0] + 0.0 * X[1] + X[2];
		u -= O;
		u /= u.norm();
		v = w.cross(u);

		tf::Matrix3x3 R(u(0), v(0), w(0), u(1), v(1), w(1), u(2), v(2), w(2));
		tf::Quaternion Q;
		R.getRotation(Q);

		visualization_msgs::Marker m;
		m.header.stamp = msg->header.stamp;
		m.header.frame_id = base_frame_;
		m.ns = "floor_plane";
		m.id = 1;
		m.type = visualization_msgs::Marker::CYLINDER;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = O(0);
		m.pose.position.y = O(1);
		m.pose.position.z = O(2);
		tf::quaternionTFToMsg(Q, m.pose.orientation);
		m.scale.x = 1.0;
		m.scale.y = 1.0;
		m.scale.z = 0.01;
		m.color.a = 0.5;
		m.color.r = 1.0;
		m.color.g = 0.0;
		m.color.b = 1.0;

		marker_pub_.publish(m);

/*
		geometry_msgs::PointStamped p;
		p.point.x = O[0];
		p.point.y = O[1];
		currentCell(p);
*/
	}

public:
	FloorPlaneMapping() :
			nh_("~") {
		nh_.param("base_frame", base_frame_, std::string("/body"));
		nh_.param("world_frame", world_frame_, std::string("/world"));
		nh_.param("max_range", max_range_, 5.0);
		nh_.param("n_samples", n_samples, 1000);
		nh_.param("tolerance", tolerance, 1.0);
		nh_.param("traverse_threshold",traverse_threshold, 1.2);

		ROS_INFO("Searching for Plane parameter z = a x + b y + c");
		ROS_INFO(
				"RANSAC: %d iteration with %f tolerance", n_samples, tolerance);
		assert(n_samples > 0);

		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneMapping::pc_callback,
				this);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",
				1);

		pCartography = new Cartography(nh_, 1.0, 10);
	}

	~FloorPlaneMapping(){
		delete pCartography;
	}

	ros::NodeHandle getNodeHanlder(){
		return nh_;
	}

	// Get a random index for the cloud point
	size_t getRandomIndex(unsigned long i) {
		size_t j = std::min((rand() / (double) RAND_MAX) * i, (double) i - 1);
		return j;
	}

	// Calculate the distance between the point and the plane
	double calcDistance(pcl::PointXYZ& point, Eigen::Vector3f& normalVector,
			double d) {
		Eigen::Vector3f t;
		t << point.x, point.y, point.z;
		return fabs(t.dot(normalVector) + d);
	}

	// Get the elapsed time from start to end
	long getElapsedTime(timeval& start, timeval& end) {
		long mtime, seconds, useconds;
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		return mtime;
	}

	// Convert the cloud point to the point stamped
	geometry_msgs::PointStamped CloudPointToPointStamped(pcl::PointXYZ& point){
		geometry_msgs::PointStamped pointStamped;
		pointStamped.point.x = point.x;
		pointStamped.point.y = point.y;
		return pointStamped;
	}

	// Update the mapping
	void updateMapping(Cell& cell) {
		// Update the cell's state and transform the points
		cell.updateState();
		cell.transformPoints(base_frame_,world_frame_);
		std::vector<geometry_msgs::PointStamped>::iterator pointsIterator;
		// Update the Cartography
		for (pointsIterator = cell.points.begin();
				pointsIterator != cell.points.end(); pointsIterator++) {
			pCartography->Update(pointsIterator->point.x, pointsIterator->point.y,
					cell.state);
		}
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "floor_plane_mapping");
	FloorPlaneMapping fp;
	ros::spin();
	return 0;
}

