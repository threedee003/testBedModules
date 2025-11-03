#include <bits/stdc++.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

/*
Real-time voxel visualization from RGBD camera stream
*/

class Voxelise {

public:
    Voxelise(ros::NodeHandle& nh)
    : it_(nh)
    {
        depth_sub_ = it_.subscribe("/camera2/camera2/depth/image_rect_raw", 1, &Voxelise::depth2PCL, this);
        info_sub_  = nh.subscribe("/camera2/camera2/depth/camera_info", 1, &Voxelise::cameraInfoCallback, this);
        rgb_sub_   = it_.subscribe("/camera2/camera2/color/image_raw", 1, &Voxelise::rgbCallback, this);
        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

        max_dist_clip = 2.0;
        voxel_size = 0.08f; // 8 cm voxel cube size

        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        viewer.reset(new pcl::visualization::PCLVisualizer("Voxelized View"));
        viewer->setPosition(0, 0);
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        viewer->setShowFPS(false);
        viewer->addCoordinateSystem();
        viewer->initCameraParameters();
        viewer->resetCamera();

        ROS_INFO("Voxel visualizer initialized (voxel size = %.2f m)", voxel_size);
    }

    ~Voxelise() {
        cv::destroyAllWindows();
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
        fx = cam_info_msg->K[0];
        fy = cam_info_msg->K[4];
        cx = cam_info_msg->K[2];
        cy = cam_info_msg->K[5];
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            rgb_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (RGB): %s", e.what());
        }
    }

    void depth2PCL(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            depth_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            depth_image.convertTo(depth_m, CV_32F, 0.001); // mm -> meters

            if (rgb_image_.empty()) return; // wait until RGB is available

            int rows = depth_m.rows;
            int cols = depth_m.cols;

            cloud->clear();
            pcl::PointXYZRGB point;

            for (int v = 0; v < rows; ++v) {
                for (int u = 0; u < cols; ++u) {
                    float z = depth_m.at<float>(v, u);
                    if (z <= 0.1 || z > max_dist_clip)
                        continue;

                    point.x = (u - cx) * z / fx;
                    point.y = (v - cy) * z / fy;
                    point.z = z;
                    point.b = rgb_image_.at<cv::Vec3b>(v, u)[0];
                    point.g = rgb_image_.at<cv::Vec3b>(v, u)[1];
                    point.r = rgb_image_.at<cv::Vec3b>(v, u)[2];
                    cloud->push_back(point);
                }
            }

            // --- VOXELIZATION ---
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            // pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            // voxel_filter.setInputCloud(cloud);
            // voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
            // voxel_filter.filter(*voxel_cloud);

            // // --- VISUALIZATION ---
            // viewer->removeAllPointClouds();
            // viewer->removeAllShapes();

            // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(voxel_cloud);

            // viewer->addPointCloud<pcl::PointXYZRGB>(voxel_cloud, rgb, "voxel_cloud");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "voxel_cloud");

            // // Optional: render cube for each voxel (real voxel geometry)
            // int id = 0;
            // for (const auto& p : voxel_cloud->points) {
            //     std::string cube_id = "v_" + std::to_string(id++);
            //     std::cout << p.b << std::endl;
            //     viewer->addCube(
            //         p.x - voxel_size/2, p.x + voxel_size/2,
            //         p.y - voxel_size/2, p.y + voxel_size/2,
            //         p.z - voxel_size/2, p.z + voxel_size/2,
            //         p.r / 255.0, p.g / 255.0, p.b / 255.0, cube_id);
            // }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
            voxel_filter.filter(*voxel_cloud);

            // Clear previous data
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Optional: show the voxel centers as points
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(voxel_cloud);
            viewer->addPointCloud(voxel_cloud, rgb, "voxel_cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "voxel_cloud");

            // Add spheres representing voxels
            float radius = voxel_size / 2.0;
            int id = 0;
            for (const auto& p : voxel_cloud->points) {
                std::string sphere_id = "sphere_" + std::to_string(id++);
                viewer->addSphere(
                    pcl::PointXYZ(p.x, p.y, p.z),
                    radius,
                    p.r / 255.0, p.g / 255.0, p.b / 255.0,
                    sphere_id
                );
                viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, sphere_id);
            }

            viewer->spinOnce(1);
            ROS_INFO_STREAM_THROTTLE(1.0, "Original: " << cloud->size() << " | Voxels: " << voxel_cloud->size());
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (Depth): %s", e.what());
        }
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;

    cv::Mat rgb_image_, depth_m, depth_image;
    double fx, fy, cx, cy, max_dist_clip, voxel_size;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_voxel_view");
    ros::NodeHandle nh;

    Voxelise viewer(nh);
    ros::spin();
    return 0;
}
