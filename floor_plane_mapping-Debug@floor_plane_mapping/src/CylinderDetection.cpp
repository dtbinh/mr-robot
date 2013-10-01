#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <sys/time.h>

class FloorPlaneRansac {
protected:
	ros::Subscriber scan_sub_;
	ros::Publisher marker_pub_;
	tf::TransformListener listener_;

	ros::NodeHandle nh_;
	std::string base_frame_;
	double max_range_;
	double tolerance;
	int n_samples;

	pcl::PointCloud<pcl::PointXYZ> lastpc_;

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

		//=================//
		// Point filtering //
		//=================//
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

		/////////////////////////////////
		// Start processing the points //
		/////////////////////////////////

		std::vector<size_t> vFittingPointsTemp;
		// Stores the fitting points for the best model
		std::vector<size_t> vFittingPoints;

		// Finding planes: z = a*x + b*y + c
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
			// Pick up 3 random points
			for (int j = 0; j < 3; j++) {
				int index = getRandomIndex((int) n);
				samplePoints[j] << lastpc_[pidx[index]].x, lastpc_[pidx[index]].y, lastpc_[pidx[index]].z;
			}
			// Calculate the plane ax+by+cz+d=0
			Eigen::Vector3f normalVector = p.cross(q);
			normalVector.normalize(); // Normalize the vector
			double d = -samplePoints[1].dot(normalVector);

			// Evaluation
			size_t score = 0;
			for (int i = 0; i < n; i++) {
				// Calculate the score for this model
				if (calcDistance(lastpc_[pidx[i]], normalVector, d)
					<= tolerance)
				{
					score++;
					vFittingPointsTemp.push_back(pidx[i]);
				}
				// Update if a better model is found
				if (score > best) {
					best = score;
					X[0] = normalVector[0] / -normalVector[2];
					X[1] = normalVector[1] / -normalVector[2];
					X[2] = d / -normalVector[2];
					vFittingPoints.push_back(pidx[i]);
				}
			}
		}
		// At the end, make sure to store the best plane estimate in X
		// X = {a,b,c}. This will be used for display
		gettimeofday(&end, NULL); // Stop the timer
		ROS_INFO("Time elapsed %ld ms", getElapsedTime(start,end));
		ROS_INFO("Score %d", (int)best);
		// END OF TODO
		ROS_INFO(
			"Extracted floor plane: z = %.2fx + %.2fy + %.2f", X[0], X[1], X[2]);

	}

public:
	FloorPlaneRansac() :
	  nh_("~") {
		  nh_.param("base_frame", base_frame_, std::string("/body"));
		  nh_.param("max_range", max_range_, 5.0);
		  nh_.param("n_samples", n_samples, 1000);
		  nh_.param("tolerance", tolerance, 1.0);

		  ROS_INFO("Searching for Plane parameter z = a x + b y + c");
		  ROS_INFO(
			  "RANSAC: %d iteration with %f tolerance", n_samples, tolerance);
		  assert(n_samples > 0);

		  // Make sure TF is ready
		  ros::Duration(0.5).sleep();

		  scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneRansac::pc_callback,
			  this);

		  // Initialize the random seed for RANSAC
		  srand(time(NULL));

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

	  bool IsInCircle(const pcl::PointXYZ& center, const double &dRadius, const pcl::PointXYZ &ptToTest, double dTolerance)
	  {
		  return ( fabs(sqrt( (ptToTest.x-center.x)*(ptToTest.x-center.x) + (ptToTest.y-center.y)*(ptToTest.y-center.y) + (ptToTest.z-center.z)*(ptToTest.z-center.z) ) - dRadius) < dTolerance);
	  }

	  void GetCenterRadiusFrom3Points2D(__out pcl::PointXYZ& center, __out double &dRadius,
		  __in const pcl::PointXYZ& A, pcl::PointXYZ& B, pcl::PointXYZ& C)
	  {
		  // Takes the bisections of AB and AC and computes their intersection
		  Eigen::Vector2f AB;
		  Eigen::Vector2f AC;
		  AB << (B.x - A.x), (B.y - A.y);
		  AC << (C.x - A.x), (C.y - A.y);
		  Eigen::Vector2f I1, I2;
		  I1 << (B.x + A.x)/2, (B.y + A.y)/2;
		  I2 << (C.x + A.x)/2, (C.y + A.y)/2;
		  // a1*x_i1+b1*y_i1+c1=0 where i1 is the middle of AB
		  // a2*x_i2+b2*y_i2+c2=0 where i2 is the middle of AC
		  double a1 = -AB.y;
		  double b1 = AB.x;
		  double c1 = -a1*I1.x-b1*I1.y;
		  double a2 = -AC.y;
		  double b2 = AC.x;
		  double c2 = -a2*I2.x-b2*I2.y;
		  center.z = (A.z+B.z+C.z)/3;
		  center.y = (a2*c1-c2*a1)/(b2*a1-a2*b1);
		  center.x = (-b1*center.y-c1)/a1;
		  radius = (sqrt( (A.x-center.x)*(A.x-center.x) + (A.y-center.y)*(A.y-center.y) + (A.z-center.z)*(A.z-center.z) )
			  + sqrt( (B.x-center.x)*(B.x-center.x) + (B.y-center.y)*(B.y-center.y) + (B.z-center.z)*(B.z-center.z) )
			  +sqrt( (C.x-center.x)*(C.x-center.x) + (C.y-center.y)*(C.y-center.y) + (C.z-center.z)*(C.z-center.z) ))/3;
	  }
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "floor_plane_Ransac");
	FloorPlaneRansac fp;

	ros::spin();
	return 0;
}