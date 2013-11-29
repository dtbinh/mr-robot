#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class ObstacleDetector{
protected:
	ros::NodeHandle m_NodeHandle;
	ros::Subscriber m_HokuyoSensorSubscriber;
	ros::Publisher m_obstaclePublisher;
	ros::Publisher m_armTwistPublisher;
	tf::TransformListener m_Listener;

	pcl::PointCloud<pcl::PointXYZ> m_basePointCloud;
	pcl::PointCloud<pcl::PointXYZ> m_obstaclePointCloud;

	std::string m_baseFrame;
	double m_maxRange;
	double m_obstacleHeight;
	double m_xRange;
	double m_yRange;
	double m_zRange;


	void hokuyoSensorCallback(const sensor_msgs::PointCloud2ConstPtr msg){
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		m_Listener.waitForTransform(m_baseFrame, msg->header.frame_id,
				msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(m_baseFrame, msg->header.stamp, temp,
				msg->header.frame_id, m_basePointCloud, m_Listener);
		m_obstaclePointCloud.header = m_basePointCloud.header;

		size_t pcSize = m_basePointCloud.size();
		std::vector<size_t> pidx;
		// Filter the points
		for(int i = 0; i < pcSize; ++i){
//			if (hypot(m_basePointCloud[i].x - m_hokuyoXOffset,m_basePointCloud[i].y)<=m_maxRange
//					&& m_basePointCloud[i].z < 2){
//				pidx.push_back(i);
//			}
			if (m_basePointCloud[i].x < m_xRange && fabs(m_basePointCloud[i].y)< m_yRange
					&& m_basePointCloud[i].z < 2){
				pidx.push_back(i);
			}
		}

		// Detect obstacles
		std::for_each(pidx.begin(),pidx.end(),[&](size_t idx){
			if(m_basePointCloud[idx].z>m_obstacleHeight){
				ROS_INFO("Found obstacle x:%f y:%f z:%f",
						m_basePointCloud[idx].x,
						m_basePointCloud[idx].y,
						m_basePointCloud[idx].z);
				m_obstaclePointCloud.points.push_back(m_basePointCloud[idx]);
			}
		});

		m_obstaclePublisher.publish(m_obstaclePointCloud);
		m_obstaclePointCloud.clear(); // Clear the buffer
	}

	void raiseArm(){

	}

public:
	ObstacleDetector():m_NodeHandle("~"){
		// Init params
		m_NodeHandle.param("base_frame", m_baseFrame, std::string("/VSV/ground"));
		m_NodeHandle.param("max_range", m_maxRange, 500.0);
		m_NodeHandle.param("obstacle_height", m_obstacleHeight, 0.2);
		m_NodeHandle.param("x_range", m_xRange, 5.0);
		m_NodeHandle.param("y_range", m_yRange, 5.0);
		m_NodeHandle.param("z_range", m_zRange, 5.0);
		// Make sure TF is ready
		ros::Duration(0.5).sleep();

		m_HokuyoSensorSubscriber = m_NodeHandle.subscribe("/vrep/hokuyoSensor",
				1,
				&ObstacleDetector::hokuyoSensorCallback,
				this);
		m_obstaclePublisher = m_NodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/obstacle",1);
		m_armTwistPublisher = m_NodeHandle.advertise<geometry_msgs::Twist>("/arm_ik/twist",1);
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "obstacle_detector");
	ObstacleDetector od;
	ros::spin();
	return 0;
}
