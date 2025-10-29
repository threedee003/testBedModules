#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>
#include <mutex>

class RealSensePCLViewer
{
public:
    RealSensePCLViewer(ros::NodeHandle& nh)
        : it_(nh), have_info_(false), viewer_("PCL RealSense Viewer")
    {
        depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
                                   &RealSensePCLViewer::depthCallback, this);
        info_sub_ = nh.subscribe("/camera/depth/camera_info", 1,
                                 &RealSensePCLViewer::infoCallback, this);

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        viewer_.setBackgroundColor(0, 0, 0);
        viewer_.addCoordinateSystem(0.1);
        viewer_.initCameraParameters();

        // Launch visualization thread
        viz_thread_ = std::thread(&RealSensePCLViewer::runVisualizer, this);

        ROS_INFO("PCL RealSense viewer initialized...");
    }

    ~RealSensePCLViewer()
    {
        viewer_.close();
        if (viz_thread_.joinable())
            viz_thread_.join();
    }

    void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        width_ = msg->width;
        height_ = msg->height;
        have_info_ = true;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if (!have_info_) return;

        cv::Mat depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        cloud_->width = width_;
        cloud_->height = height_;
        cloud_->is_dense = false;
        cloud_->points.resize(width_ * height_);

        const float depth_scale = 0.001f; // mm -> meters
        for (int v = 0; v < height_; ++v)
        {
            for (int u = 0; u < width_; ++u)
            {
                uint16_t d = depth.at<uint16_t>(v, u);
                pcl::PointXYZ& pt = cloud_->at(u, v);
                if (d == 0) {
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }

                float z = d * depth_scale;
                pt.x = (u - cx_) * z / fx_;
                pt.y = (v - cy_) * z / fy_;
                pt.z = z;
            }
        }
    }

private:
    void runVisualizer()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        while (!viewer_.wasStopped() && ros::ok())
        {
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                *local_cloud = *cloud_;
            }

            if (!viewer_.updatePointCloud(local_cloud, "cloud"))
                viewer_.addPointCloud(local_cloud, "cloud");

            viewer_.spinOnce(10);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }

    // ROS
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;

    // Intrinsics
    bool have_info_;
    double fx_, fy_, cx_, cy_;
    int width_, height_;

    // PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::visualization::PCLVisualizer viewer_;
    std::thread viz_thread_;
    std::mutex cloud_mutex_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_pcl_viewer");
    ros::NodeHandle nh;

    RealSensePCLViewer viewer(nh);
    ros::spin();

    return 0;
}
