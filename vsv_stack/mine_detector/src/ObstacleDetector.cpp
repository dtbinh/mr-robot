#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <utility>
#include <numeric>
#include <math.h>

// Precision conversion
#define prec_1(x) static_cast<float>(static_cast<int>(x*10))/10
#define prec_2(x) static_cast<float>(static_cast<int>(x*20))/20

class ObstacleDetector{
protected:
	ros::NodeHandle m_NodeHandle;
	ros::Subscriber m_HokuyoSensorSubscriber;
	ros::Subscriber m_twistSubscriber;
	ros::Publisher m_obstaclePublisher;
	ros::Publisher m_armTwistPublisher;
	tf::TransformListener m_Listener;

	pcl::PointCloud<pcl::PointXYZ> m_basePointCloud;
	pcl::PointCloud<pcl::PointXYZ> m_worldPointCloud;
	pcl::PointCloud<pcl::PointXYZ> m_obstaclePointCloud;
	geometry_msgs::PointStamped m_toolRefPS;
	geometry_msgs::PointStamped m_toolBasePS;
	geometry_msgs::Twist m_twist;

	std::string m_baseFrame;
	std::string m_worldFrame;
	double m_maxRange;
	double m_obstacleHeight;
	double m_xRange;
	double m_yRange;
	double m_zRange;
	bool m_enableAvoidance;


	void hokuyoSensorCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		m_Listener.waitForTransform(m_baseFrame, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(m_baseFrame, msg->header.stamp, temp,
				msg->header.frame_id, m_basePointCloud, m_Listener);
		//
		m_Listener.waitForTransform(m_worldFrame, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(m_worldFrame, msg->header.stamp, temp,
				msg->header.frame_id, m_worldPointCloud, m_Listener);

		m_obstaclePointCloud.header.frame_id = m_worldPointCloud.header.frame_id;

		m_Listener.waitForTransform(m_baseFrame, "/VSV/Tool", msg->header.stamp,ros::Duration(1.0));
		m_Listener.transformPoint(m_worldFrame, m_toolRefPS, m_toolBasePS);

		size_t pcSize = m_basePointCloud.size();
		std::vector<size_t> pidx;
		// Filter the points
		for(int i = 0; i < pcSize; ++i){
			if (m_basePointCloud[i].x < m_xRange && fabs(m_basePointCloud[i].y)< m_yRange
					&& m_basePointCloud[i].z < 2){
				pidx.push_back(i);
			}
		}
		pcl::PointXYZ p;
		// Detect obstacles
		std::for_each(pidx.begin(),pidx.end(),[&](size_t idx){
			if(m_basePointCloud[idx].z>m_obstacleHeight){
				p.x = prec_1(m_worldPointCloud[idx].x);
				p.y = prec_1(m_worldPointCloud[idx].y);
				p.z = prec_2(m_worldPointCloud[idx].z);
				if(!isPointExist(m_obstaclePointCloud,p)){
					ROS_INFO("Found obstacle x:%f y:%f z:%f",p.x,p.y,p.z);
					m_obstaclePointCloud.points.push_back(p);
				}
//				raiseArm();
			}
		});

		ROS_INFO("Tool x:%f y:%f z:%f",m_toolBasePS.point.x,m_toolBasePS.point.y,m_toolBasePS.point.z);
		if(isDangerous(m_obstaclePointCloud,m_toolBasePS.point)){
			std::cout<<"Dangerous"<<std::endl;
			raiseArm();
		}
		m_obstaclePublisher.publish(m_obstaclePointCloud);
//		m_obstaclePointCloud.clear(); // Clear the buffer
//		std::cout<<m_obstaclePointCloud.size()<<std::endl; // Display the point cloud size
	}

	// Check if the point is in the point cloud
	bool isPointExist(const pcl::PointCloud<pcl::PointXYZ>& pcl, const pcl::PointXYZ& p){
		size_t pcSize = pcl.points.size();
		for(int i = 0; i < pcSize; ++i){
			if(fabs(p.x - pcl[i].x)<=0.2 && fabs(p.y - pcl[i].y)<=0.2
					&& fabs(p.z - pcl[i].z)<0.1){
				return true;
			}
		}
		return false;
	}

	// Check if the position is near any point in the point cloud
	bool isDangerous(const pcl::PointCloud<pcl::PointXYZ>& pcl, const geometry_msgs::Point& p){
		size_t pcSize = pcl.points.size();
		for(int i = 0; i < pcSize; ++i){
			// TODO determine dangerous range
//			ROS_INFO("Obstacle x:%f y:%f z:%f",pcl[i].x,pcl[i].y,pcl[i].z);
//			ROS_INFO("Dist x:%f y:%f z:%f",fabs(p.x - pcl[i].x),fabs(p.y - pcl[i].y),fabs(p.z - pcl[i].z));
			if(fabs(p.x - pcl[i].x)<=0.5 && fabs(p.y - pcl[i].y)<=0.8 && fabs(p.z - pcl[i].z)<= 0.5){
				return true;
			}
		}
		return false;
	}

	void twistCallback(const geometry_msgs::Twist& msg){
		m_twist = std::move(msg);
	}

	void raiseArm(){
		geometry_msgs::Twist t;
		t.linear.z=0.1;
		m_armTwistPublisher.publish(t);
	}

public:
	ObstacleDetector():m_NodeHandle("~"),m_enableAvoidance(false){
		// Init params
		m_NodeHandle.param("base_frame", m_baseFrame, std::string("/VSV/ground"));
		m_NodeHandle.param("world_frame", m_worldFrame, std::string("/world"));
		m_NodeHandle.param("max_range", m_maxRange, 500.0);
		m_NodeHandle.param("obstacle_height", m_obstacleHeight, 0.2);
		m_NodeHandle.param("x_range", m_xRange, 5.0);
		m_NodeHandle.param("y_range", m_yRange, 5.0);
		m_NodeHandle.param("z_range", m_zRange, 5.0);
		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		m_HokuyoSensorSubscriber = m_NodeHandle.subscribe("/vrep/hokuyoSensor",1,
				&ObstacleDetector::hokuyoSensorCallback, this);
		m_twistSubscriber = m_NodeHandle.subscribe("/vsv_driver/twistCommand",1,
				&ObstacleDetector::twistCallback, this);

		m_obstaclePublisher = m_NodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/obstacle",1);
		m_armTwistPublisher = m_NodeHandle.advertise<geometry_msgs::Twist>("/arm_ik/twist",1);

		m_toolRefPS.header.frame_id="/VSV/Tool";
		m_toolRefPS.point.x=0;
		m_toolRefPS.point.y=0;
		m_toolRefPS.point.z=-0.25;
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "obstacle_detector");
	ObstacleDetector od;
	ros::spin();
	return 0;
}
