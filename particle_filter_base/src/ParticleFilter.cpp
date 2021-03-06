#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <particle_filter_base/Function.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>

#include <stdlib.h>
#include <sys/time.h>
#include <random>

static double mapSize_ = 1.0;
static double dCellSize_ = 1.0;
static unsigned int uiCellSize_ = 10;
static double toleranceForDEMParsing = 0.05;
static double sigma2 = 1.0;

static std::default_random_engine generator;
static std::normal_distribution<double> distribution(0.0,0.01);

class Timer {
private:

	timeval startTime;

public:

	void start(){
		gettimeofday(&startTime, NULL);
	}

	double stop(){
		timeval endTime;
		long seconds, useconds;
		double duration;

		gettimeofday(&endTime, NULL);

		seconds  = endTime.tv_sec  - startTime.tv_sec;
		useconds = endTime.tv_usec - startTime.tv_usec;

		duration = seconds + useconds/1000000.0;

		return duration;
	}

};


namespace particle_filter_base {

    class Particle {
        protected:
            double importance;
			 // here some state
    		double x;
    		double y;

        public:
    		Particle()	{}
            Particle(double _x, double _y) : x(_x), y(_y), importance(0.0) {}
            // Some initialisation
            Particle(double _x, double _y, double i) : x(_x), y(_y), importance(i) {}

            // The prediction stage need to apply some control
			// Input : linear velocity and angular velocity
            void applyControl(const geometry_msgs::Twist &twist, double deltaT) {
				// We assume instant speed change
				x += twist.linear.x*deltaT + distribution(generator);
				y += twist.linear.y*deltaT + distribution(generator);
            }

            // The observation stage need to update the particle importance
			// TODO
            void updateImportance(const std::pair<double, double>  &vRobotPosition_fromMeasurements) {
            	double xm = vRobotPosition_fromMeasurements.first;
            	double ym = vRobotPosition_fromMeasurements.second;
				importance = 1/(sqrt(2*M_PI*sigma2))*exp(-((x-xm)*(x-xm)+(y-ym)*(y-ym))/(2*sigma2));
            }

            double getImportance() const {
                return importance;
            }

            void setImportance(double w) {
                importance = w;
            }

            // Convert the particle state to a ROS pose for display in RVIZ
            void toPoseMessage(geometry_msgs::Pose & pose) {
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = importance;
                pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
            }

			double getX()	{return x;}
			double getY()	{return y;}

			static double distance2D(const Particle& p1, const Particle &p2)
			{
				double dx=p2.x-p1.x;
				double dy=p2.y-p1.y;
				return sqrt(dx*dx+dy*dy);
			}
    };

    class ParticleFilter {
        protected:
            std::vector<Particle> particles;

        public:
            ParticleFilter() {}

            void initialise(size_t num_particles, double x, double spread, double mapSize) {
                particles.resize(num_particles);
				int squareSide = sqrt(num_particles);	// The map is a square
				// Choose a uniform initial distribution of points
				for (size_t i = 0; i < squareSide; i++) {
					for (size_t j = 0; j < squareSide; j++) {
						particles[squareSide*i+j] = Particle(double(mapSize)*double(i)/double(squareSide),
								double(mapSize)*double(j)/double(squareSide));
						particles[squareSide*i+j].setImportance(1.0/particles.size());
					}
				}
				for (size_t i = squareSide*squareSide; i < num_particles; i++) {
					particles[i] = Particle(double(mapSize)*random()/(double)RAND_MAX,
							double(mapSize)*random()/(double)RAND_MAX);
					particles[i].setImportance(1.0/particles.size());
				}


            }

			void predict(const geometry_msgs::Twist &twist, double deltaT) {
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i].applyControl(twist, deltaT);
                }
            }

            void update(const std::pair<double, double>  &vRobotPosition_fromMeasurements) {
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i].updateImportance(vRobotPosition_fromMeasurements);
                }
                resample();
            }

            void resample() {
                Function cdf, inv_cdf;
                double sum = 0.0;
                // First build the cdf
                for (size_t i = 0; i < particles.size(); i++) {
                    sum += particles[i].getImportance();
                    cdf.set(i,sum);
                }
                // Now normalise
                cdf = cdf / sum;
                inv_cdf = cdf.inverse();

                std::vector<Particle> new_particles(particles.size());
                for (size_t i = 0; i < particles.size(); i++) {
                    double u = random() / (double)RAND_MAX;
                    double j = inv_cdf(u);
					int j1 = int(j);
					int j2 = j1+1;
					// Linear interpolation
					if(j2>=particles.size())
					{
						new_particles[i] = particles[j1];
					}
					else
					{
						if(Particle::distance2D(particles[j1], particles[j2]) > mapSize_/2)
							new_particles[i] = particles[j1];
						else	// This should be the most common case
						{
							double delta = j-double(j1);
							double deltaX = particles[j2].getX()-particles[j1].getX();
							double deltaY = particles[j2].getY()-particles[j1].getY();
							double deltaI = particles[j2].getImportance()-particles[j1].getImportance();
							new_particles[i] = Particle(particles[j1].getX()+deltaX*delta,
								particles[j1].getY()+deltaY*delta,
								particles[j1].getImportance()+deltaI*delta);
						}
					}
                }
                particles = new_particles;
            }

            void publishPoseArray(const std_msgs::Header & header, ros::Publisher & pub) {
                geometry_msgs::PoseArray poses;
                poses.header = header;
                poses.poses.resize(particles.size());
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i].toPoseMessage(poses.poses[i]);
                }
                pub.publish(poses);
            }
    };

    class ParticleFilterLocalisation {
        protected:
            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            ros::Subscriber twist_sub_;
            ros::Subscriber scan_sub_;
            ros::Publisher pose_pub_;
            image_transport::Subscriber DEM_sub_, DEM_cov_sub_;
            tf::TransformListener listener_;

            std::string base_frame_;
            double max_range_;
            int num_particles_;
            double initial_spread_;
			// Lame variable name...
			int K_;
			double tolerance_;

            pcl::PointCloud<pcl::PointXYZ> lastpc_;

            geometry_msgs::Twist twist;
            // OpenCV matrices to receive the Digital Elevation Map (dem) and
            // its uncertainty/covariance
            bool dem_received, dem_cov_received;
            cv::Mat dem, dem_cov;
            double dem_x_orig, dem_y_orig, dem_scale;
            ParticleFilter pf;

            bool demToWorld(const cv::Point2i & P, cv::Point2f & R) {
                R = cv::Point2f(dem_x_orig + P.x*dem_scale,dem_y_orig + P.y*dem_scale);
                return true;
            }

            bool worldToDem(const cv::Point2f & P, cv::Point2i & R) {
                R = cv::Point2i(round((P.x - dem_x_orig)/dem_scale), 
                        round((P.y - dem_y_orig)/dem_scale));
                if (R.x < 0) return false;
                if (R.y < 0) return false;
                if (R.x >= dem.cols) return false;
                if (R.y >= dem.rows) return false;
                return true;
            }

			// Get a random index for the cloud point
			static inline size_t GetRandomIndex(unsigned long i)
			{
				return std::min((rand() / (double) RAND_MAX) * i, (double) i - 1);
			}

            inline double ConvertIndexToWorldCoord(int i)
             {
                     return (double(i)/uiCellSize_)*dCellSize_;
             }

            inline int ConvertWorldCoordToIndex(double d)
            {
            	return d*dCellSize_/double(uiCellSize_);
            }

        protected: // ROS Callbacks
			
            // Find if mat(m, n) is such that the surrounding points follow the distribution in vConfig
            bool foundConfiguration(const std::vector<pcl::PointXYZ> &vConfig, cv::Mat &mat, int m, int n)
            {
            	int i = m+ConvertWorldCoordToIndex(vConfig[0].x);
            	int j = n+ConvertWorldCoordToIndex(vConfig[0].y);
            	if(m+i>mat.size().width || m+i<0 || n+j>mat.size().width || n+j<0)
            		return false;

            	double reference = mat.at<float>(m, n);
            	for(auto it=vConfig.begin(); it != vConfig.end(); ++it)
            	{
            		i = ConvertWorldCoordToIndex(it->x);
            		j = ConvertWorldCoordToIndex(it->y);
            		if(m+i>mat.size().width || m+i<0 || n+j>mat.size().width || n+j<0)
            			continue;
            		if(fabs((mat.at<float>(m+i,n+i)-reference)-it->z)>toleranceForDEMParsing)
            			return false;
            	}
            	return true;
            }

            void DetermineRobotPositionFromDEM(std::vector<pcl::PointXYZ> &vMeasurements, std::pair<double, double> &robotPosition, double yaw)
            {
            	// Rescaling
            	double reference = vMeasurements[0].z;
            	for(auto it=vMeasurements.begin(); it!=vMeasurements.end(); ++it)
            	{
            			it->z-=reference;
            			double x = it->x;
            			double y = it->y;
            			it->x = cos(yaw)*x-sin(yaw)*y;
            			it->y = sin(yaw)*x+cos(yaw)*y;
            	}
            	for(int i = 0; i < dem.size().height; i++)
            	{
            		for(int j = 0; j < dem.size().width; j++)
            		{
            			if(foundConfiguration(vMeasurements, dem, i, j))
            			{
            				robotPosition.first = ConvertIndexToWorldCoord(i);
            				robotPosition.second = ConvertIndexToWorldCoord(j);
            			}
            		}
            	}

            }

            void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
                static Timer sTimer;
				static double sDeltaT = 0.0;

				std_msgs::Header header;
                pcl::PointCloud<pcl::PointXYZ> temp;
                pcl::fromROSMsg(*msg, temp);
                // Make sure the point cloud is in the base-frame
                listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
                header.stamp = msg->header.stamp;
                header.frame_id = base_frame_;

				if(sDeltaT == -INT_MAX)
				{
					sTimer.start();
					sDeltaT = 0.0;
				}
				else
				{
					sDeltaT = sTimer.stop();
					sTimer.start();
				}
				if (sDeltaT > 10)
					sDeltaT = 0;

                //
                pf.predict(twist, sDeltaT);
                if (dem_received ) {
					dem_received = false;
					// Take K points in the lastpc_
					std::vector<pcl::PointXYZ> vMeasurements;
					for(int i=0; i<K_; ++i)
						vMeasurements.push_back(lastpc_[GetRandomIndex(lastpc_.size())]);
					std::pair<double, double> robotPosition;
					// Get yaw
					listener_.waitForTransform("/world", base_frame_,
							msg->header.stamp, ros::Duration(1.0));
					tf::StampedTransform transform;
					listener_.lookupTransform("/world", base_frame_,
							msg->header.stamp, transform);
					auto Q = transform.getRotation();
					tf::Matrix3x3 M;
					M.setRotation(Q);
					double roll, pitch, yaw;
					M.getRPY(roll, pitch, yaw);
					DetermineRobotPositionFromDEM(vMeasurements, robotPosition, yaw);
                    pf.update(robotPosition);
                }
                pf.publishPoseArray(header, pose_pub_);
            }

            void dem_callback(const sensor_msgs::ImageConstPtr& msg) {
                dem_received = true;
                // In theory, this would require a mutex, but because the
                // callbacks are called by ROS, we know that only one of them
                // is running at a time. 
                dem = cv_bridge::toCvShare(msg,"rgba8")->image;
            }

            void dem_cov_callback(const sensor_msgs::ImageConstPtr& msg) {
                dem_cov_received = true;
                // In theory, this would require a mutex, but because the
                // callbacks are called by ROS, we know that only one of them
                // is running at a time. 
                dem_cov = cv_bridge::toCvShare(msg,"rgba8")->image;
            }

            void twist_callback(const geometry_msgs::TwistStamped& msg){
            	twist = msg.twist;
            }

        public:
            ParticleFilterLocalisation() : nh_("~"), it_(nh_) {
                std::string transport = "raw";
                nh_.param("transport",transport,transport);
                nh_.param("base_frame",base_frame_,std::string("/body"));
                nh_.param("max_range",max_range_,5.0);
                nh_.param("num_particles",num_particles_,100);
                nh_.param("initial_spread",initial_spread_,1.0);
				nh_.param("mapSize",mapSize_, 10.0);
				
				nh_.param("K",K_, 50);
				nh_.param("tolerance",tolerance_, 0.005);

                // DEM parameters
                nh_.param("dem_x_orig",dem_x_orig,0.0);
                nh_.param("dem_y_orig",dem_y_orig,0.0);
                nh_.param("dem_scale",dem_scale,1.0);
                nh_.param("toleranceForDEMParsing",toleranceForDEMParsing,0.05);


                pf.initialise(num_particles_,0.0,initial_spread_, mapSize_);

                dem_received = dem_cov_received = false;

                // Make sure TF is ready
                ros::Duration(0.5).sleep();

                twist_sub_ = nh_.subscribe("/vrep/twistStatus",1,&ParticleFilterLocalisation::twist_callback,this);
                scan_sub_ = nh_.subscribe("scans",1,&ParticleFilterLocalisation::pc_callback,this);
                pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles",1);
                DEM_sub_ = it_.subscribe<ParticleFilterLocalisation>("dem",1, &ParticleFilterLocalisation::dem_callback,this,transport);
                DEM_cov_sub_ = it_.subscribe<ParticleFilterLocalisation>("dem_covariance",1, &ParticleFilterLocalisation::dem_cov_callback,this,transport);

            }

    };
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"particle_filter");
    particle_filter_base::ParticleFilterLocalisation fp;

    ros::spin();
    return 0;
}

