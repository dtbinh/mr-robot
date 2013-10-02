﻿#include <ros/ros.h>
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
#include <map>
#include <vector>

#define THRESHOLD_TO_PARSE_PLANE	10
#define THRESHOLD_TO_ACCEPT_CIRCLE_FOUND	10

class CylinderDetection
{
protected:
	ros::Subscriber m_Subscriber
		ros::Publisher marker_pub_;
	tf::TransformListener listener_;

	ros::NodeHandle m_NodeHandle;
	std::string m_strBaseFrame;
	double m_dMaxRange;
	double m_dTolerance;
	int m_nSamples;

	pcl::PointCloud<pcl::PointXYZ> m_PointCloud;
	//! Key : plane section equation (z = Key)
	//! Value : a vector of indices of the points of m_MapPlaneSections which belong to (z = Key)
	std::map<double, std::vector<size_t>> m_MapPlaneSections;
	// Step used to discretize the z coordinate of m_PointCloud
	double m_dDiscretizationPrecision;

protected:
	// ROS Callbacks

	void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg(*msg, temp);
		// Make sure the point cloud is in the base-frame
		listener_.waitForTransform(m_strBaseFrame, msg->header.frame_id,
			msg->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(m_strBaseFrame, msg->header.stamp, temp,
			msg->header.frame_id, m_PointCloud, listener_);

		//=================//
		// Point filtering //
		//=================//
		unsigned int n = temp.size();
		// First count the useful points
		for (unsigned int i = 0; i < n; i++) {
			float x = temp[i].x;
			float y = temp[i].y;
			float d = hypot(x, y);
			if (d < 1e-2) {
				// Bogus point, ignore
				continue;
			}
			x = m_PointCloud[i].x;
			y = m_PointCloud[i].y;
			d = hypot(x, y);
			if (d > m_dMaxRange) {
				// too far, ignore
				continue;
			}
			double z = floor(m_PointCloud[i].z / m_dDiscretizationPrecision)*m_dDiscretizationPrecision;
			auto it = m_MapPlaneSections.find(z);
			if(it != m_MapPlaneSections.end())
				it->second.push_back(i);
			else
				m_MapPlaneSections[z] = std::vector<size_t>(1, i);
		}

		/////////////////////////////////
		// Start processing the points //
		/////////////////////////////////

		std::map<double, std::pair<pcl::PointXYZ, double>> mapCirclesInPlaneSections;

		for(auto it = m_MapPlaneSections.begin(); it != m_MapPlaneSections.end(); ++it)
		{
			std::vector<size_t> vFittingPointsTemp;
			// Stores the fitting points for the best model
			std::vector<size_t> vFittingPoints;

			size_t best = 0;
			// Stores the center for the best model
			pcl::PointXYZ center;
			// Stores the radius for the best model
			double radius;

			auto pvPoints = &(it->second);	// pvPoint has type std::vector<size_t>*
			n = pvPoints->size();	// # points in the section 'z = it->first'
			if(n < THRESHOLD_TO_PARSE_PLANE)
				continue;

			ROS_INFO("%d useful points in the section at z = %.2f", n, it->first);

			////////////////
			// RANSACK START
			////////////////
			for (unsigned int i = 0; i < (unsigned) m_nSamples; ++i)
			{
				vFittingPointsTemp.clear();
				pcl::PointXYZ samplePoints[3];
				pcl::PointXYZ centerForCurrentIteration;
				double radiusForCurrentIteration;

				// Pick up 3 random points with indices taken from pvPoints 
				int buffer[3] = {-1, -1, -1};	// Used to guarantee that we pick 3 different numbers
				for (int j = 0; j < 3; j++)
				{
					bool bNumberAlreadyFound = true;
					int index = GetRandomIndex(n);
					while(bNumberAlreadyFound)
					{
						for(int i = 0; i <= j; ++i)
						{
							if(index == buffer[i])
							{
								index = GetRandomIndex(n);
								i = -1;
							}
						}
						bNumberAlreadyFound = false;
						buffer[j] = index;
					}

					const pcl::PointXYZ &pt = m_PointCloud[pvPoints->at(index)];
					samplePoints[j](pt.x, pt.y, pt.z;
				}

				// Calculate the circle which intersects the 3 points chosen
				GetCenterRadiusFrom3Points2D(centerForCurrentIteration, radiusForCurrentIteration, samplePoints[0], samplePoints[1], samplePoints[2]);

				// Evaluation
				size_t score = 0;
				for (auto itPtIndex = pvPoints->begin(); itPtIndex!=pvPoints->end(); ++itPtIndex)
				{
					// Calculate the score for this model
					if (IsInCircle(centerForCurrentIteration, radiusForCurrentIteration, m_PointCloud[*itPtIndex], m_dTolerance)
					{
						score++;
						vFittingPointsTemp.push_back(*itPtIndex);
					}
				}

				// Update if a better model is found
				if (score > best)
				{
					best = score;
					center = centerForCurrentIteration;
					radius = radiusForCurrentIteration
						vFittingPoints = vFittingPointsTemp;
				}
			}
			// END OF RANSAC FOR THIS SECTION
			// ^___^

			if (score < THRESHOLD_TO_ACCEPT_CIRCLE_FOUND)
				continue;

			mapCirclesInPlaneSections[it->first] = std::make_pair(center, radius);

			ROS_INFO("Circle found at z = %.2f Score %d Center (%.2f, %.2f, %.2f) Radius %.2f", it->first, (int)best, center.x, center.y, center.z, radius );
		}

		// CONCAT THE RESULTS FROM mapCirclesInPlaneSections
		if(it.size < 3)
			return;	// Not enough acceptable sections to consider that there is a cylinder
		for(auto it = mapCirclesInPlaneSections.begin(); it!=mapCirclesInPlaneSections.end(); ++it)
		{
			// TODO
			// ^___^
		}
	}

public:
	CylinderDetection() :
		m_NodeHandle("~") {
			m_NodeHandle.param("base_frame", m_strBaseFrame, std::string("/body"));
			m_NodeHandle.param("max_range", m_dMaxRange, 5.0);
			m_NodeHandle.param("n_samples", m_nSamples, 1000);
			m_NodeHandle.param("tolerance", m_dTolerance, 1.0);

			ROS_INFO("Searching for vertical cylinders");
			ROS_INFO(
				"RANSAC: %d iteration with %f tolerance", m_nSamples, tolerance);
			assert(m_nSamples > 0);

			// Make sure TF is ready
			ros::Duration(0.5).sleep();

			m_Subscriber = m_NodeHandle.subscribe("scans", 1, &CylinderDetection::pc_callback, this);

			// Initialize the random seed for RANSAC
			srand(time(NULL));

	}

	// Get a random index for the cloud point
	__forceinline size_t GetRandomIndex(unsigned long i)
	{
		return std::min((rand() / (double) RAND_MAX) * i, (double) i - 1);
	}

	// Calculate the distance between the point and the plane
	double GetDistanceToPlane(pcl::PointXYZ& point, Eigen::Vector3f& normalVector,
		double d) {
			Eigen::Vector3f t;
			t << point.x, point.y, point.z;
			return fabs(t.dot(normalVector) + d);
	}

	// Get the elapsed time from start to end
	long GetElapsedTime(timeval& start, timeval& end)
	{
		long mtime, seconds, useconds;
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		return mtime;
	}

	__forceinline bool IsInCircle(const pcl::PointXYZ& center, const double &dRadius, const pcl::PointXYZ &ptToTest, double dTolerance)
	{
		return ( fabs(sqrt( (ptToTest.x-center.x)*(ptToTest.x-center.x) + (ptToTest.y-center.y)*(ptToTest.y-center.y) + (ptToTest.z-center.z)*(ptToTest.z-center.z) ) - dRadius) < dTolerance);
	}

	void GetCenterRadiusFrom3Points2D(pcl::PointXYZ& center, double &dRadius,
		const pcl::PointXYZ& A, const pcl::PointXYZ& B, const pcl::PointXYZ& C)
	{
		// Takes the bisections of AB and AC and computes their intersection
		Eigen::Vector2f AB;
		Eigen::Vector2f AC;
		AB << (B.x - A.x), (B.y - A.y);
		AC << (C.x - A.x), (C.y - A.y);
		Eigen::Vector2f I1, I2;
		I1 << (B.x + A.x)/2, (B.y + A.y)/2;
		I2 << (C.x + A.x)/2, (C.y + A.y)/2;

		// a1*(y-y_i1)-b1*(x-x_i1)=0 where i1 is the middle of AB
		// a2*(y-y_i2)-b2*(x-x_i2)=0 where i2 is the middle of AC
		double a1 = -AB.y;
		double b1 = AB.x;
		double a2 = -AC.y;
		double b2 = AC.x;
		center.z = (A.z+B.z+C.z)/3;
		if(a1 == 0)
		{
			center.x = I1.x;
			center.y = b2*(center.x-I2.x)/a2+I2.y;	// We assume AB and AC are not //. Hence a1 and a2 can't be both null.
		}
		else
		{
			center.x = (-b2*I2.x+a2*I1.x/a1+a2*(I2.y-I1.y))/(a2*b1/a1-b2);
			center.y = b1*(center.x-I1.x)/a1+I1.y;
		}
		dRadius = (sqrt( (A.x-center.x)*(A.x-center.x) + (A.y-center.y)*(A.y-center.y) + (A.z-center.z)*(A.z-center.z) )
			+ sqrt( (B.x-center.x)*(B.x-center.x) + (B.y-center.y)*(B.y-center.y) + (B.z-center.z)*(B.z-center.z) )
			+sqrt( (C.x-center.x)*(C.x-center.x) + (C.y-center.y)*(C.y-center.y) + (C.z-center.z)*(C.z-center.z) ))/3;
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "CylinderDetection");
	CylinderDetection fp;

	ros::spin();
	return 0;
}