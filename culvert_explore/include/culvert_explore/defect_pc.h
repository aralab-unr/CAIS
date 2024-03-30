#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Dense>

using namespace std;

class defect_pc
{
    public:
        defect_pc();
        // ~defect_pc();
        void pc_seg_cb(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        /*  objects to sync pointcloud and defect image subscribers  */
        message_filters::Subscriber<sensor_msgs::Image> defect_seg_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        
        /*  subscriber topics   */
        string defect_seg_topic;
        string cloud_topic;

        /*  publishers  */
        ros::Publisher defect_pc_pub_;
        ros::Publisher defect_map_pub_;

        /*  publishers topics   */
        string defect_pc_topic;
        string defect_map_topic;
        bool publish_defect_pc;

        /*  tf frame for defect map */
        string global_frame;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        /*  defect map  */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud;

        bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& source_frame,
                             const std::string& target_frame)
        {
            geometry_msgs::TransformStamped transform;
            try
            {
                transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
                return false;
            }

            // Transform the point cloud
            Eigen::Affine3d eigen_transform = transformToEigen(transform);
            pcl::transformPointCloud(*cloud, *cloud, eigen_transform);

            return true;
        }

        Eigen::Affine3d transformToEigen(const geometry_msgs::TransformStamped& transform)
        {
            Eigen::Affine3d eigen_transform;
            eigen_transform.translation() << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
            Eigen::Quaterniond quat(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
            eigen_transform.linear() = quat.toRotationMatrix();
            return eigen_transform;
        }

};