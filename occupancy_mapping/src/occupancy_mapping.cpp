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
#include <opencv2/ml/ml.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <sys/time.h>
#include "Cartography.h"
#include "Cell.h"
#include "DME.h"

static double ALPHA	= 1.0;
static double BETA = 2.0;
static double Z_THRESHOLD = 0.4;
static double belief_mod = 3.0;

const double PI  =3.141592653589793238462;

class FloorPlaneMapping {
protected:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	ros::Publisher pcl_pub_;

	tf::TransformListener listener_;

	std::string base_frame_;
	std::string world_frame_;

	int n_samples;
	double max_range_;
	double tolerance;
	double traverse_threshold; // Angle threshold to determine if traversable
	double normal_estimation_radius; // Normal estimation radius in meters
	double step_function_parameter;

	pcl::PointCloud<pcl::PointXYZ> lastpc_;

	Cartography *m_pCartography;
	DME *m_pDME;

protected:
	// ROS Callbacks

	void UpdateCartographAndDME(float x, float y, float z, int state)
	{
		double logOdd = 0.0;
		double distanceToRobot = hypot(x, y);
		if(state == Traversable)
			logOdd = belief_mod;
		else if(state == NonTraversable)
			logOdd = -belief_mod;
		else
			logOdd = 0;
		// Step function such that f(0+)=1 and f(+infinite)->0+, f(0-)=-1 and f(-infinite)->0-
		// The function chosen is f(x)=tanh(ALPHA*param/x^BETA)
		double d=logOdd*tanh(ALPHA*step_function_parameter/pow(distanceToRobot, BETA));
		ROS_INFO("Update CARTO");
		m_pCartography->Update(x,y,d);
		ROS_INFO("Update");
		m_pDME->Update(x,y,z);
	}

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
		 * RANSAC
		 */		
		n = pidx.size();
		size_t best = 0;

		Eigen::Vector3f normalVector;
		std::vector<size_t> inliersIndex; // index for inliers of the plane
		std::vector<size_t> outliersIndex; // index for outliers
//		ROS_INFO("%d useful points out of %d", (int)n, (int)temp.size());
		for (unsigned int i = 0; i < (unsigned) n_samples; i++) 
		{
			if (n == 0)
				break;
			Eigen::Vector3f samplePoints[3];
			// Initialize the random seed
			// srand(time(NULL));
			// Pick up 3 random points
			for (int j = 0; j < 3; j++) 
			{
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
			std::vector<size_t> tempInliersIndex;
			std::vector<size_t> tempOutliersIndex;
			for (int i = 0; i < n; i++) {
				// Calculate the score for this model
				if (calcDistance(lastpc_[pidx[i]], normalVector, d)
					<= tolerance)
				{
						tempInliersIndex.push_back(i);
						score++;
				}
				else
					tempOutliersIndex.push_back(i);

				// Update if a better model is found
				if (score > best)
				{
					best = score;
					inliersIndex = tempInliersIndex;
					outliersIndex = tempOutliersIndex;
				}
			}
		}
		// End of RANSAC


		/*
		 * Update and mapping
		 */

		int pidx_size = inliersIndex.size();

		Eigen::Vector3f zVector;
		double angle;
		zVector << 0, 0, 1;
		// Get the angle between the normalVector and the Z axis
		angle = acos(
				normalVector.dot(zVector) / (normalVector.norm() * zVector.norm()));
		// Set the threshold
		if(fabs(angle) > PI/2)
			angle=PI-fabs(angle);

		if (fabs(angle) <= traverse_threshold)
		{
			// Update the inliers
			for(int i = 0; i<pidx_size; ++i)
			{
				float x = cloudPtr->points[inliersIndex[i]].x;
				float y = cloudPtr->points[inliersIndex[i]].y;
				float z = cloudPtr->points[inliersIndex[i]].z;
				UpdateCartographAndDME(x,y,z,Traversable);
				//ROS_INFO("Traversable");
			}
			pidx_size = outliersIndex.size();
			for(int i = 0; i<pidx_size; ++i)
			{
				float x = cloudPtr->points[outliersIndex[i]].x;
				float y = cloudPtr->points[outliersIndex[i]].y;
				float z = cloudPtr->points[outliersIndex[i]].z;
				if(z > Z_THRESHOLD)
					UpdateCartographAndDME(x,y,z, NonTraversable);
				else
					UpdateCartographAndDME(x,y,z, Unknown);
				//ROS_INFO("NonTraversable/Unknown");
			}
		}
		else
		{
			for(int i = 0; i<pidx_size; ++i)
			{
				float x = cloudPtr->points[inliersIndex[i]].x;
				float y = cloudPtr->points[inliersIndex[i]].y;
				float z = cloudPtr->points[inliersIndex[i]].z;
				UpdateCartographAndDME(x,y,z,NonTraversable);
				//ROS_INFO("NonTraversable");
			}
			pidx_size = outliersIndex.size();
			for(int i = 0; i<pidx_size; ++i)
			{
				float x = cloudPtr->points[outliersIndex[i]].x;
				float y = cloudPtr->points[outliersIndex[i]].y;
				float z = cloudPtr->points[outliersIndex[i]].z;
				UpdateCartographAndDME(x,y,z, Unknown);
				//ROS_INFO("Unknown");
			}
		}

		m_pCartography->PublishImage();
		m_pDME->PublishToFile();

		/*
		 * SVM
		 */
/*		CvSVM svm;
		CvSVMParams params;

		// Set the params
	    params.svm_type    = CvSVM::C_SVC;
	    params.kernel_type = CvSVM::RBF; // RBF Kernel: exp(-gamma*|u-v|^2)
	    params.C = 1;
	    params.gamma = 0.5;
	    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

	    // Training
	    // {height}:{occupancy}
	    cv::Mat trainData = m_pDME->getMat()->reshape(0,1);
	    cv::Mat labels = m_pCartography->getMat()->reshape(0,1);
	    svm.train(trainData,labels,cv::Mat(),cv::Mat(),params);
	    svm.save("svm_model.xml");

	    // Predict
	    cv::Mat samples(n_samples,1,CV_32F);
	    cv::Mat results(n_samples,1,CV_32F);
	    for (unsigned int i = 0; i < (unsigned) n_samples; i++) {
	    	samples.at<float>(i,1) = lastpc_[pidx[i]].z;
	    }
	    svm.predict(samples,results);

	    // Publish the results
	    std::vector<size_t> obstacleIndex;
	    pcl::PointCloud<pcl::PointXYZ> pcl;
	    for (unsigned int i = 0; i < (unsigned) n_samples; i++) {
	    	if (results.at<float>(i,1) == NonTraversable){
	    		pcl.push_back(lastpc_[pidx[i]]);
	    	}
	    }
	    pcl_pub_.publish(pcl);*/
	    /*
	     * End of SVM
	     */
	}

public:
	FloorPlaneMapping() :
			nh_("~")
	{
		nh_.param("base_frame", base_frame_, std::string("/body"));
		nh_.param("world_frame", world_frame_, std::string("/world"));
		nh_.param("max_range", max_range_, 5.0);
		nh_.param("normal_estimation_radius",normal_estimation_radius,0.03);
		nh_.param("traverse_threshold",traverse_threshold, 0.3);
		nh_.param("step_function_parameter",step_function_parameter, 2.0);
		nh_.param("n_samples", n_samples, 1000);
		nh_.param("tolerance", tolerance, 1.0);
		nh_.param("alpha", ALPHA, 1.0);
		nh_.param("beta", BETA, 2.0);
		nh_.param("z_threshold", Z_THRESHOLD, 0.4);
		nh_.param("belief_mod", belief_mod, 3.0);

		ROS_INFO("Mapping");
		assert(n_samples > 0);

		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneMapping::pc_Callback,
				this);
//		pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("obstacles",1);

		m_pCartography = new Cartography(nh_, 1.0, 10);
		m_pDME = new DME(1.0, 10);
	}

	~FloorPlaneMapping()
	{
		delete m_pCartography;
		delete m_pDME;
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
};

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "occupancy_mapping");
	FloorPlaneMapping fp;
	ros::spin();
	return 0;
}

