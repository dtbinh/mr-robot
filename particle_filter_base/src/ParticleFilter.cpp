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
#include "DEM.h"


namespace particle_filter_base {

    class Particle {
        protected:
            // here some state
    		double x;
    		double y;
            double importance;
        public:
            Particle() : importance(0.0) {}
            // Some initialisation
            Particle(double i) : importance(i) {}

            // The prediction stage need to apply some control
            void applyControl(double u) {
            }

            // The observation stage need to update the particle importance
            void updateImportance(double z) {
            }

            double getImportance() const {
                return importance;
            }

            void setImportance(double w) {
                importance = w;
            }

            // Convert the particle state to a ROS pose for display in RVIZ
            void toPoseMessage(geometry_msgs::Pose & pose) {
                pose.position.x = 0.0;
                pose.position.y = 0.0;
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
            }
    };

    class ParticleFilter {
        protected:
            std::vector<Particle> particles;

        public:
            ParticleFilter() {}

            void initialise(size_t num_particles, double x, double spread) {
                particles.resize(num_particles);
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i] = Particle(x + spread*(-1.0 + 2.0*random()/(double)RAND_MAX));
                    particles[i].setImportance(1.0/particles.size());
                }
            }

            void predict(double u) {
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i].applyControl(u);
                }
            }

            void update(double z) {
                for (size_t i = 0; i < particles.size(); i++) {
                    particles[i].updateImportance(z);
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
                    int j = (int)round(inv_cdf(u));
                    new_particles[i] = particles[i];
                    new_particles[j].setImportance(1.0/particles.size());
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

            pcl::PointCloud<pcl::PointXYZ> lastpc_;

            geometry_msgs::Twist twist;
            // OpenCV matrices to receive the Digital Elevation Map (dem) and
            // its uncertainty/covariance
            bool dem_received, dem_cov_received;
            cv::Mat dem, dem_cov;
            double dem_x_orig, dem_y_orig, dem_scale;
            ParticleFilter pf;
            DEM* pDEM;

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

        protected: // ROS Callbacks

            void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
                std_msgs::Header header;
                pcl::PointCloud<pcl::PointXYZ> temp;
                pcl::fromROSMsg(*msg, temp);
                // Make sure the point cloud is in the base-frame
                listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
                header.stamp = msg->header.stamp;
                header.frame_id = base_frame_;

                //
                pf.predict(0.0);
                if (dem_received && dem_cov_received) {
                    pf.update(lastpc_[0].z);
                }
                pf.publishPoseArray(header, pose_pub_);
            }

            void dem_callback(const sensor_msgs::ImageConstPtr& msg) {
                dem_received = true;
                // In theory, this would require a mutex, but because the
                // callbacks are called by ROS, we know that only one of them
                // is running at a time. 
                dem = cv_bridge::toCvShare(msg,"mono8")->image;
            }

            void dem_cov_callback(const sensor_msgs::ImageConstPtr& msg) {
                dem_cov_received = true;
                // In theory, this would require a mutex, but because the
                // callbacks are called by ROS, we know that only one of them
                // is running at a time. 
                dem_cov = cv_bridge::toCvShare(msg,"mono8")->image;
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

                // DEM parameters
                nh_.param("dem_x_orig",dem_x_orig,0.0);
                nh_.param("dem_y_orig",dem_y_orig,0.0);
                nh_.param("dem_scale",dem_scale,1.0);

                pf.initialise(num_particles_,0.0,initial_spread_);

                dem_received = dem_cov_received = false;

                // Make sure TF is ready
                ros::Duration(0.5).sleep();

                twist_sub_ = nh_.subscribe("/vrep/twistStatus",1,&ParticleFilterLocalisation::twist_callback,this);
                scan_sub_ = nh_.subscribe("scans",1,&ParticleFilterLocalisation::pc_callback,this);
                pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles",1);
                DEM_sub_ = it_.subscribe<ParticleFilterLocalisation>("dem",1, &ParticleFilterLocalisation::dem_callback,this,transport);
                DEM_cov_sub_ = it_.subscribe<ParticleFilterLocalisation>("dem_covariance",1, &ParticleFilterLocalisation::dem_cov_callback,this,transport);

                pDEM = new DEM(1.0, 10);
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

