#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
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

const double PI=3.141592653589793238462;
static const char * svm_output = "/tmp/svm_model.xml";

class FloorPlaneMapping {
protected:
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	ros::Subscriber joy_sub_;
	ros::Publisher pcl_pub_;
	ros::Publisher marker_pub_;

	tf::TransformListener listener_;

	std::string base_frame_;
	std::string world_frame_;

	int n_samples;
	double max_range_;
	double tolerance;
	double traverse_threshold; // Angle threshold to determine if traversable
	double normal_estimation_radius; // Normal estimation radius in meters
	double step_function_parameter;
	double ALPHA;
	double BETA;
	double Z_THRESHOLD;
	double belief_mod;

	pcl::PointCloud<pcl::PointXYZ> basePC;
	pcl::PointCloud<pcl::PointXYZ> worldPC;  // Point cloud in the world frame
	pcl::PointCloud<pcl::PointXYZ> obstaclePC; // Point cloud of obstacles
	pcl::PointCloud<pcl::PointXYZ> testPC;

	Cartography *m_pCartography;
	DME *m_pDME;

	// SVM
	CvSVM svm;
	CvSVMParams params;
	bool isSVMOn;

protected:
	/*
	 * CALLBACK:
	 * PointCloud
	 */
	void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg){
		/**
		 * Transformation of the point clouds
		 */
		pcl::PointCloud<pcl::PointXYZ> temp;

		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		listener_.waitForTransform(base_frame_, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
				msg->header.frame_id, basePC, listener_);

		// cloudPtr -> point cloud in the world frame
		listener_.waitForTransform(world_frame_, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(world_frame_, msg->header.stamp, temp,
				msg->header.frame_id, worldPC, listener_);

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
			x = basePC[i].x;
			y = basePC[i].y;
			d = hypot(x, y);
			if (d > max_range_) {
				// too far, ignore
				continue;
			}
			pidx.push_back(i);
		}
		
		/*
		 * ==========================
		 * RANSAC
		 * ==========================
		 */		
		n = pidx.size();
		size_t best = 0;
		double X[3] = { 0, 0, 0 };

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
				samplePoints[j] << basePC[pidx[index]].x, basePC[pidx[index]].y, basePC[pidx[index]].z;
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
				if (calcDistance(basePC[pidx[i]], normalVector, d)
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
					X[0] = normalVector[0] / -normalVector[2];
					X[1] = normalVector[1] / -normalVector[2];
					X[2] = d / -normalVector[2];
				}
			}
		}

		// Floor plane marker
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
		 * ==========================
		 * End of RANSAC
		 * ==========================
		 */


		/*
		 * ==========================
		 * Mapping
		 * ==========================
		 */
		Eigen::Vector3f zVector;
		double angle;
		zVector << 0, 0, 1;
		// Get the angle between the normalVector and the Z axis
		angle = acos(normalVector.dot(zVector) / (normalVector.norm() * zVector.norm()));
		// Set the threshold
		if(fabs(angle) > PI/2)
			angle=PI-fabs(angle);

		// Update the state
		testPC.clear();
		CellState inlierState;
		CellState outlierState;
		CellState outlierAltState = Unknown;
		if (fabs(angle) <= traverse_threshold) {
			inlierState = Traversable;
			outlierState = NonTraversable;
		}else{
			inlierState = NonTraversable;
			outlierState = Unknown;
		}

		// Update inliers
		int pidx_size = inliersIndex.size();
		for (int i = 0; i < pidx_size; ++i) {
			float x = worldPC[inliersIndex[i]].x;
			float y = worldPC[inliersIndex[i]].y;
			float z = worldPC[inliersIndex[i]].z;
			testPC.push_back(worldPC[inliersIndex[i]]);
			UpdateCartographAndDME(x, y, z, inlierState);
		}

		// Update outliers
		pidx_size = outliersIndex.size();
		for (int i = 0; i < pidx_size; ++i) {
			float x = worldPC[outliersIndex[i]].x;
			float y = worldPC[outliersIndex[i]].y;
			float z = worldPC[outliersIndex[i]].z;
			float z_base = basePC[outliersIndex[i]].z; // z in the base frame
			if (z_base > Z_THRESHOLD)
				UpdateCartographAndDME(x, y, z, outlierState);
			else
				UpdateCartographAndDME(x, y, z, outlierAltState);
		}

		pcl_pub_.publish(testPC);
		// Publish the results
		m_pCartography->PublishImage();
		m_pDME->PublishToFile();

		/*
		 * ==========================
		 * End of Mapping
		 * ==========================
		 */

		/*
		 * ==========================
		 * SVM
		 * ==========================
		 */

		obstaclePC.clear();

		// TODO
		cv::Mat inputMat;
		if(isSVMOn){
			for (int i=0;i<n;++i){
				inputMat.at<float>(0,0) = worldPC[pidx[i]].z;
				float result = svm.predict(inputMat);
				if(result == -1){
					ROS_INFO("%f",result);
				}
				obstaclePC.push_back(worldPC[pidx[i]]);
			}
			// Publish the point cloud
			pcl_pub_.publish(obstaclePC);
		}
		/*
		 * ==========================
	     * End of SVM
	     * ==========================
	     */
	}

	/**
	 * CALLBACK
	 * JOY
	 */
	void joy_Callback(const sensor_msgs::Joy::ConstPtr& joy){
		// Button "A"
		if(joy->buttons[0]==1){
			// Begin training the SVM
			ROS_INFO("BUTTON \"A\" PRESSED\n Training SVM");
			// Reshape and transpose the matrix
		    cv::Mat heightMat = m_pDME->getMat()->reshape(0,1).t();
		    cv::Mat varMat = m_pDME->getVarMat()->reshape(0,1).t();
		    cv::Mat cartoMat = m_pCartography->getMat()->reshape(0,1).t();

		    // Training Data
		    cv::Mat trainingData(heightMat.rows,heightMat.cols+varMat.cols,CV_32FC1);
		    heightMat.col(0).copyTo(trainingData.col(0));
		    varMat.col(0).copyTo(trainingData.col(1));
		    // Convert labels
		    cv::Mat labels(cartoMat.rows,cartoMat.cols,CV_32FC1);
		    // Map log odd values to labels
		    ROS_INFO("%d",labels.rows);
		    for(int i=0;i<labels.rows;++i){
		    	float label;
		    	if(cartoMat.at<float>(i,0)>0.5) // Traversable
		    		label = 1;
		    	else
		    		label = -1;
		    	labels.at<float>(i,0)=label;
		    }
		    svm.train(trainingData,labels,cv::Mat(),cv::Mat(),params);
		    // Save the model
		    svm.save(svm_output);
		    ROS_INFO("SVM saved to %s",svm_output);
		}
		// Button "X"
		if(joy->buttons[2]==1){
			if(!isSVMOn){
				isSVMOn = true;
				ROS_INFO("SVM on");
				svm.load(svm_output);
			}
		}
		// Button "Y"
		if(joy->buttons[3]==1){
			if(isSVMOn){
				isSVMOn =false;
				ROS_INFO("SVM off");
			}
		}
	}

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
		m_pCartography->Update(x,y,d);
		m_pDME->Update(x,y,z);
	}
public:
	FloorPlaneMapping() :
			nh_("~"),svm()
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

		ROS_INFO("Running");
		ROS_INFO("Press \"A\" button to train the svm");
		ROS_INFO("Press \"X\"/\"Y\" button to turn on/off the svm prediction");
		assert(n_samples > 0);

		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		// Subscribers
		scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneMapping::pc_Callback,
				this);

		joy_sub_ = nh_.subscribe("/joy",10, &FloorPlaneMapping::joy_Callback,this);
		// Publishers
		pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("obstacles",1);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane", 1);
		// Mapping classes
		m_pCartography = new Cartography(nh_, 1.0, 10);
		m_pDME = new DME(1.0, 10);

		// Point Cloud
		obstaclePC.header.frame_id = world_frame_;
		testPC.header.frame_id = world_frame_;

		// SVM Parameters
		isSVMOn = false;
		params.svm_type = CvSVM::C_SVC;
		params.kernel_type = CvSVM::RBF; // RBF Kernel: exp(-gamma*|u-v|^2)
//		params.C = 1;
//		params.gamma = 0.5;
		params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 500, 1e-6);
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

