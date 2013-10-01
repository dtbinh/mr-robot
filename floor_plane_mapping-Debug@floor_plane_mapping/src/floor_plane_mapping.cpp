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
	int n_samples;

	pcl::PointCloud<pcl::PointXYZ> lastpc_;
	Cartography *m_pCartography;

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
			Eigen::Vector3f normalVector = p.cross(q);
			normalVector.normalize(); // Normalize the vector
			double d = -samplePoints[1].dot(normalVector);

			// Evaluation
			size_t score = 0;
			for (int i = 0; i < n; i++) {
				// Calculate the score for this model
				if (calcDistance(lastpc_[pidx[i]], normalVector, d)
						<= tolerance) {
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

		geometry_msgs::PointStamped p;
		p.point.x = O[0];
		p.point.y = O[1];
		currentCell(p);
		//
//		transformPoint();
	}

public:
	FloorPlaneMapping() :
			nh_("~") {
		nh_.param("base_frame", base_frame_, std::string("/body"));
		nh_.param("world_frame", world_frame_, std::string("/world"));
		nh_.param("max_range", max_range_, 5.0);
		nh_.param("n_samples", n_samples, 1000);
		nh_.param("tolerance", tolerance, 1.0);

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

		m_pCartography = new Cartography(nh_, 1.0, 10);
	}

	~FloorPlaneMapping(){
		delete m_pCartography;
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

	/**
	 * Transform the point from the robot's base
	 */
	void transformPoint(const ros::TimerEvent& event){
//	void transformPoint(){
	  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
	  tf::TransformListener listener(ros::Duration(10));
	  std::string world_frame = "world";
	  geometry_msgs::PointStamped robot_point;
	  geometry_msgs::PointStamped base_point;
	  robot_point.header.frame_id = base_frame_;

	  //we'll just use the most recent transform available for our simple example
	  robot_point.header.stamp = ros::Time();

	  robot_point.point.x = 0.0;
	  robot_point.point.y = 0.0;
	  robot_point.point.z = 0.0;

	  try{
	    listener.waitForTransform(world_frame, robot_point.header.frame_id,
				robot_point.header.stamp, ros::Duration(3.0));
	    listener.transformPoint(world_frame, robot_point, base_point);

	    ROS_INFO("%s: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",base_frame_.c_str(),
	        robot_point.point.x, robot_point.point.y, robot_point.point.z,world_frame.c_str(),
	        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
	  }
	  catch(tf::TransformException& ex){
		  ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s",
				  base_frame_.c_str(),
				  world_frame.c_str(),
				  ex.what());
	  }
	}

	/**
	 * Transform the point from the base to the world
	 */
	void transformPointToWorld(geometry_msgs::PointStamped& basePoint, geometry_msgs::PointStamped& worldPoint) {
		tf::TransformListener listener(ros::Duration(10));
		basePoint.header.frame_id = base_frame_;
		try {
			listener.waitForTransform(world_frame_, basePoint.header.frame_id,
					basePoint.header.stamp, ros::Duration(3.0));
			listener.transformPoint(world_frame_, basePoint, worldPoint);
		} catch (tf::TransformException& ex) {
			ROS_ERROR(
					"Received an exception trying to transform a point from \"%s\" to \"%s\": %s",
					base_frame_.c_str(),
					world_frame_.c_str(),
					ex.what());
		}
	}

	// TODO
	// Get the state of the cell
	// according to the normal vector of the cell
	CellState getCellState(Eigen::Vector3f& normalVector){
		CellState cellState = Unknown;
		Eigen::Vector3f zVector;
		double angle;
		zVector<<0,0,1;
		// Get the angle between the normalVector and the Z unit vector
		angle = acos(normalVector.dot(zVector)/(normalVector.norm()*zVector.norm()));
		// TODO
		// Set the threshold
		if (fabs(angle) > 1.2)
			cellState = Traversable;
		else
			cellState = NonTraversable;
		return cellState;
	}

	// TODO
	// Determine cells' accuracy
	Cell currentCell(geometry_msgs::PointStamped& point){
		Cell cell;
		geometry_msgs::PointStamped worldPoint;
		transformPointToWorld(point, worldPoint);
		cell.x = round(double(worldPoint.point.x));
		cell.y = round(double(worldPoint.point.y));
		//		currentCell.cellState = getCellState();
		m_pCartography->Update(cell.x, cell.y, cell.cellState);
		ROS_INFO("---- Point x: %.2f y: %.2f",point.point.x,point.point.y);
		ROS_INFO("---- Point x: %.2f y: %.2f",worldPoint.point.x,worldPoint.point.y);
		ROS_INFO("---- Current cell x: %.2f y: %.2f",cell.x,cell.y);

		return cell;
	}

};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "floor_plane_mapping");
	FloorPlaneMapping fp;
/*	ros::Timer timer = fp.getNodeHanlder().createTimer(ros::Duration(1.0),
			&FloorPlaneMapping::transformPoint,
			&fp);*/
	ros::spin();
	return 0;
}

