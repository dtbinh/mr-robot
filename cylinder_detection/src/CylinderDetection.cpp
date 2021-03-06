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

#define _DO_LINEAR_REGRESSION
//#define _VERBOSE
#define __forceinline inline

static int THRESHOLD_LOWER_TO_PARSE_PLANE = 10;
static int THRESHOLD_UPPER_TO_PARSE_PLANE = 600;
static int THRESHOLD_TO_ACCEPT_CIRCLE_FOUND = 10;
static int MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER = 4;
static double TOLERANCE_FOR_PLANE_CONCATENATION = 0.01;
static double MAXIMUM_RADIUS = 1.0;

//==================//
// Helper functions //
//==================//

// Get a random index for the cloud point
static __forceinline size_t GetRandomIndex(unsigned long i)
{
	return std::min((rand() / (double) RAND_MAX) * i, (double) i - 1);
}

// Calculate the distance between 2 points
static __forceinline double GetDistanceBetweenPoints3D(const pcl::PointXYZ& A, const pcl::PointXYZ& B)
{
	return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y)+(A.z-B.z)*(A.z-B.z));
}

// Calculate the distance between 2 points (only x and y coordiantes are considered)
static __forceinline double GetDistanceBetweenPoints2D(const pcl::PointXYZ& A, const pcl::PointXYZ& B)
{
	return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
}


// Calculate the distance between the point and the plane
static double GetDistanceToPlane(const pcl::PointXYZ& point, const Eigen::Vector3f& normalVector, double d)
{
	Eigen::Vector3f t;
	t << point.x, point.y, point.z;
	return fabs(t.dot(normalVector) + d);
}

static __forceinline bool IsInCircle(const pcl::PointXYZ& center, const double &dRadius, const pcl::PointXYZ &ptToTest, double dTolerance)
{
	return ( fabs(sqrt( (ptToTest.x-center.x)*(ptToTest.x-center.x) + (ptToTest.y-center.y)*(ptToTest.y-center.y) ) - dRadius) < dTolerance);
}

// Output : center and dRadius
static void GetCenterRadiusFrom3Points2D(pcl::PointXYZ& center, double &dRadius,
								  const pcl::PointXYZ& A, const pcl::PointXYZ& B, const pcl::PointXYZ& C)
{

	Eigen::MatrixXf M1(3,3);
	M1(0,0) = A.x*A.x+A.y*A.y;
	M1(0,1) = A.y;
	M1(0,2) = 1;
	M1(1,0) = B.x*B.x+B.y*B.y;
	M1(1,1) = B.y;
	M1(1,2) = 1;
	M1(2,0) = C.x*C.x+C.y*C.y;
	M1(2,1) = C.y;
	M1(2,2) = 1;

	Eigen::MatrixXf M2(3,3);
	M2(0,0) = A.x;
	M2(0,1) = A.y;
	M2(0,2) = 1;
	M2(1,0) = B.x;
	M2(1,1) = B.y;
	M2(1,2) = 1;
	M2(2,0) = C.x;
	M2(2,1) = C.y;
	M2(2,2) = 1;

	center.x = M1.determinant()/(2*M2.determinant());

	M1(0,1) = A.x*A.x+A.y*A.y;
	M1(0,0) = A.x;
	M1(0,2) = 1;
	M1(1,1) = B.x*B.x+B.y*B.y;
	M1(1,0) = B.x;
	M1(1,2) = 1;
	M1(2,1) = C.x*C.x+C.y*C.y;
	M1(2,0) = C.x;
	M1(2,2) = 1;

	center.y = M1.determinant()/(2*M2.determinant());
	/*
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
	double a1 = -AB.y();
	double b1 = AB.x();
	double a2 = -AC.y();
	double b2 = AC.x();
	if(a1 == 0)
	{
		center.x = I1.x();
		center.y = b2*(center.x-I2.x())/a2+I2.y();	// We assume AB and AC are not //. Hence a1 and a2 can't be both null.
	}
	else
	{
		center.x = (-b2*I2.x()+a2*I1.x()/a1+a2*(I2.y()-I1.y()))/(a2*b1/a1-b2);
		center.y = b1*(center.x-I1.x())/a1+I1.y();
	}
	*/

	center.z = (A.z+B.z+C.z)/3;
	dRadius = (sqrt( (A.x-center.x)*(A.x-center.x) + (A.y-center.y)*(A.y-center.y) + (A.z-center.z)*(A.z-center.z) )
		+ sqrt( (B.x-center.x)*(B.x-center.x) + (B.y-center.y)*(B.y-center.y) + (B.z-center.z)*(B.z-center.z) )
		+sqrt( (C.x-center.x)*(C.x-center.x) + (C.y-center.y)*(C.y-center.y) + (C.z-center.z)*(C.z-center.z) ))/3;
}

// Get the elapsed time from start to end
static long GetElapsedTime(timeval& start, timeval& end)
{
	long mtime, seconds, useconds;
	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
	return mtime;
}

// ^__^
// We are done with the helper functions

// CylinderDetection : THE class which does all the cylinder recognition job .
// Since it is supposed to be used internally only for the node created in this file, we did not bother
// declaring a header for this class.
//===================================
class CylinderDetection
{
protected:
	ros::Subscriber m_Subscriber;
	ros::Publisher marker_pub_;
	tf::TransformListener listener_;

	ros::NodeHandle m_NodeHandle;
	std::string m_strBaseFrame;
	double m_dMaxRange;
	double m_dRansacTolerance;
	int m_nSamples;

	pcl::PointCloud<pcl::PointXYZ> m_PointCloud;
	//! Key : plane section equation (z = Key)
	//! Value : a vector of indices of the points of m_MapPlaneSections which belong to (z = Key)
	std::map<double, std::vector<size_t>> m_MapPlaneSections;
	// Step used to discretize the z coordinate of m_PointCloud
	double m_dDiscretizationPrecision;

protected:
	//==============//
	// ROS Callback //
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
		m_MapPlaneSections.clear();
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

		// Stores the circle found for each section (if a circle was found)
		std::map<double, std::pair<pcl::PointXYZ, double>> mapCirclesInPlaneSections;

		for(auto it = m_MapPlaneSections.begin(); it != m_MapPlaneSections.end(); ++it)
		{
			std::vector<size_t> vFittingPointsTemp;
			// Stores the indices of the fitting points for the best model
			std::vector<size_t> vFittingPoints;

			size_t best = 0;
			// Stores the center for the best model
			pcl::PointXYZ center;
			// Stores the radius for the best model
			double radius;



			// Reference on the vector of points indices which belong to the section pointed by 'it'
			auto pvPoints = &(it->second);	// pvPoint has type std::vector<size_t>*
			n = pvPoints->size();	// # points in the section 'z = it->first'

#ifdef _VERBOSE
			ROS_INFO("%d useful points in the section at z = %.2f", n, it->first);
#endif

			if(n < THRESHOLD_LOWER_TO_PARSE_PLANE)
				continue;
			if(n > THRESHOLD_UPPER_TO_PARSE_PLANE)
				continue;


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
					samplePoints[j] = pcl::PointXYZ(pt.x, pt.y, pt.z);
				}

				// Calculate the circle which intersects the 3 points chosen
				GetCenterRadiusFrom3Points2D(centerForCurrentIteration, radiusForCurrentIteration, samplePoints[0], samplePoints[1], samplePoints[2]);

#ifdef _VERBOSE
			/*ROS_INFO("Circle from A(%.5f, %.5f, %.5f) B(%.5f, %.5f, %.5f) C(%.5f, %.5f, %.5f) - Center (%.2f, %.2f, %.2f) Radius %.2f", samplePoints[0].x, samplePoints[0].y, samplePoints[0].z,
					samplePoints[1].x, samplePoints[1].y, samplePoints[1].z,
					samplePoints[2].x, samplePoints[2].y, samplePoints[2].z,
					centerForCurrentIteration.x, centerForCurrentIteration.y, centerForCurrentIteration.z, radiusForCurrentIteration );
					*/
#endif


				// Evaluation
				size_t score = 0;
				for (auto itPtIndex = pvPoints->begin(); itPtIndex!=pvPoints->end(); ++itPtIndex)
				{
					// Calculate the score for this model
					if (IsInCircle(centerForCurrentIteration, radiusForCurrentIteration, m_PointCloud[*itPtIndex], m_dRansacTolerance))
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
					radius = radiusForCurrentIteration;
						vFittingPoints = vFittingPointsTemp;
				}
			}
			// END OF RANSAC FOR THIS SECTION
			// ^___^

			if (best < THRESHOLD_TO_ACCEPT_CIRCLE_FOUND)
				continue;

			if(radius > MAXIMUM_RADIUS)
				continue;

#ifdef _DO_LINEAR_REGRESSION
			//////////////////////////////////////////
			// Linear regression to refine the results
			Eigen::MatrixXf A(3,3);
            Eigen::MatrixXf B(3,1);
			
			// Initialize the default values of A and B
			for(unsigned int i = 0; i < 3; i++)
			{
				B(i, 0) = 0;
				for(unsigned int j = 0; j < 3; j++)
					A(i, j) = 0;
			}

			A(2,2) = n;
			for (auto itPtIndex = vFittingPoints.begin(); itPtIndex != vFittingPoints.end(); ++itPtIndex)
			{
				// Assign x,y to the coordinates of the point we are considering.
				double x = m_PointCloud[*itPtIndex].x;
				double y = m_PointCloud[*itPtIndex].y;
			
				A(0,0) = A(0,0) + x*x;
				A(0,1) = A(0,1) + x*y;
				A(0,2) = A(0,2) + x;
				A(1,0) = A(1,0) + x*y;
				A(1,1) = A(1,1) + y*y;
				A(1,2) = A(1,2) + y;
				A(2,0) = A(2,0) + x;
				A(2,1) = A(2,1) + y;

				B(0,0) = B(0,0) + x*(x*x+y*y);
				B(1,0) = B(1,0) + y*(x*x+y*y);
				B(2,0) = B(2,0) + x*x+y*y;
			}
			
			Eigen::MatrixXf X = A.colPivHouseholderQr().solve(B);

			center.x = -X(0)/2.0;
			center.y = -X(1)/2.0;
			radius = sqrt(4*X(2)+X(0)*X(0)+X(1)*X(1))/2;
#endif

			// At last add the circle found to mapCirclesInPlaneSections
			mapCirclesInPlaneSections[it->first] = std::make_pair(center, radius);

#ifdef _VERBOSE
			ROS_INFO("Circle found at z = %.2f with score %d and Center (%.2f, %.2f) Radius %.2f", it->first, (int)best, center.x, center.y, radius );
#endif
		}

#ifdef _VERBOSE
			ROS_INFO("%d planes with a valid circle inside", mapCirclesInPlaneSections.size() );
#endif

		//==================================================
		// CONCAT THE RESULTS FROM mapCirclesInPlaneSections
		if(mapCirclesInPlaneSections.size() < MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER)
			return;	// Not enough acceptable sections to consider that there is a cylinder

		pcl::PointXYZ meanCenter = pcl::PointXYZ(0, 0, 0);
		double meanRadius = 0;
		int numSections = 1;
		double dBottomCap = DBL_MAX;
		double dTopCap;
		for(auto it = mapCirclesInPlaneSections.begin(); it!=mapCirclesInPlaneSections.end(); ++it)
		{
			auto itNext = it;
			++itNext;
#ifdef _VERBOSE
			ROS_INFO("Distance between centers %.2f Difference in radius %.2f Section count %d", GetDistanceBetweenPoints2D(it->second.first, itNext->second.first), fabs(it->second.second-itNext->second.second), numSections );
#endif
			if(itNext != mapCirclesInPlaneSections.end())
			{
				if(GetDistanceBetweenPoints2D(it->second.first, itNext->second.first) < TOLERANCE_FOR_PLANE_CONCATENATION && fabs(it->second.second-itNext->second.second) < TOLERANCE_FOR_PLANE_CONCATENATION)
				{
					if(dBottomCap == DBL_MAX)
						dBottomCap = it->first;
					++numSections;
					meanCenter.x += ((itNext->second.first.x + it->second.first.x)/2.0);
					meanCenter.y += ((itNext->second.first.y + it->second.first.y)/2.0);
					meanRadius += ((itNext->second.second+it->second.second)/2.0);
				}
				else
				{
					dTopCap = it->first;
					meanCenter.x /= numSections;
					meanCenter.y /= numSections;
					meanRadius /= numSections;
					if(numSections >= MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER)
					{
						geometry_msgs::PointStamped robotSpaceCenter;
						robotSpaceCenter.point.x = meanCenter.x;
						robotSpaceCenter.point.y = meanCenter.y;
						listener_.waitForTransform("/world", m_strBaseFrame,
												msg->header.stamp, ros::Duration(1.0));
						tf::StampedTransform transform;
						listener_.lookupTransform("/world", m_strBaseFrame,
												msg->header.stamp, transform);

						// if(numSections < MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER) Not enough acceptable sections to consider that there is a cylinder
						auto worldSpaceCenter = transform * tf::Vector3(robotSpaceCenter.point.x, robotSpaceCenter.point.y, robotSpaceCenter.point.z);
						ROS_INFO("Cylinder found: Center (%.2f, %.2f) Radius %.2f Bottom z = %.2f Top z = %.2f", meanCenter.x, meanCenter.y, meanRadius, dBottomCap, dTopCap);
						ROS_INFO("Cylinder found (World space coordinates): Center (%.2f, %.2f) Radius %.2f Bottom z = %.2f Top z = %.2f", worldSpaceCenter.x(), worldSpaceCenter.y(), meanRadius, dBottomCap, dTopCap);
					}
					meanCenter = pcl::PointXYZ(0, 0, 0);
					meanRadius = 0;
					numSections = 1;
					dBottomCap = DBL_MAX;
				}
			}
			else
			{
				dTopCap = it->first;
				meanCenter.x /= numSections;
				meanCenter.y /= numSections;
				meanRadius /= numSections;
				if(numSections >= MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER)
				{
					geometry_msgs::PointStamped robotSpaceCenter;
					robotSpaceCenter.point.x = meanCenter.x;
					robotSpaceCenter.point.y = meanCenter.y;
					listener_.waitForTransform("/world", m_strBaseFrame,
							msg->header.stamp, ros::Duration(1.0));
					tf::StampedTransform transform;
					listener_.lookupTransform("/world", m_strBaseFrame,
							msg->header.stamp, transform);

					// if(numSections < MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER) Not enough acceptable sections to consider that there is a cylinder
					auto worldSpaceCenter = transform * tf::Vector3(robotSpaceCenter.point.x, robotSpaceCenter.point.y, robotSpaceCenter.point.z);
					ROS_INFO("Cylinder found: Center (%.2f, %.2f) Radius %.2f Bottom z = %.2f Top z = %.2f", meanCenter.x, meanCenter.y, meanRadius, dBottomCap, dTopCap);
					ROS_INFO("Cylinder found (World space coordinates): Center (%.2f, %.2f) Radius %.2f Bottom z = %.2f Top z = %.2f", worldSpaceCenter.x(), worldSpaceCenter.y(), meanRadius, dBottomCap, dTopCap);

				}
			}

			// ^___^
		}
		
	}

public:
	CylinderDetection() :
		m_NodeHandle("~")
	{
			m_NodeHandle.param("base_frame", m_strBaseFrame, std::string("/body"));
			m_NodeHandle.param("max_range", m_dMaxRange, 5.0);
			m_NodeHandle.param("n_samples", m_nSamples, 1000);
			m_NodeHandle.param("ransac_tolerance", m_dRansacTolerance, 1.0);
			m_NodeHandle.param("parse_plane_lower_bound_threshhold", THRESHOLD_LOWER_TO_PARSE_PLANE, 10);
			m_NodeHandle.param("parse_plane_upper_bound_threshhold", THRESHOLD_UPPER_TO_PARSE_PLANE, 600);
			m_NodeHandle.param("accept_circle_threshhold", THRESHOLD_TO_ACCEPT_CIRCLE_FOUND, 10);
			m_NodeHandle.param("discretization_precision", m_dDiscretizationPrecision, 0.025);
			m_NodeHandle.param("min_num_valid_sections_to_accept_cylinder", MIN_NUM_VALID_SECTIONS_TO_ACCEPT_CYLINDER, 4);
			m_NodeHandle.param("tolerane_for_plane_concatenation", TOLERANCE_FOR_PLANE_CONCATENATION, 0.01);
			m_NodeHandle.param("maximum_radius", MAXIMUM_RADIUS, 1.0);

			ROS_INFO("Searching for vertical cylinders");
			ROS_INFO("RANSAC: %d iteration with %f tolerance", m_nSamples, m_dRansacTolerance);
			assert(m_nSamples > 0);

			// Make sure TF is ready
			ros::Duration(0.5).sleep();

			m_Subscriber = m_NodeHandle.subscribe("/vrep/depthSensor", 1, &CylinderDetection::pc_callback, this);

			// Initialize the random seed for RANSAC
			srand(time(NULL));

	}
};

///////////////////////
// M   M  AA  I N  N //
// MM MM A  A I NN N //
// M M M AAAA I N NN //
// M   M A  A I N  N //
///////////////////////

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "cylinder_detection");
	CylinderDetection node;
	ros::spin();
	return 0;
}
