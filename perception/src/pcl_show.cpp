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
using namespace std;

class PCLView{

public:
    PCLView(ros::NodeHandle& nh)  
    : it_(nh)
    {   
        depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &PCLView::depth2PCL, this);
        info_sub_ = nh.subscribe("/camera/depth/camera_info", 1, &PCLView::cameraInfoCallback, this);
        rgb_sub_ = it_.subscribe("/camera/color/image_raw", 1, &PCLView::rgbCallback, this);
        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

        max_dist_clip = 2.0;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        viewer.reset(new pcl::visualization::PCLVisualizer("PCL View"));
        viewer->setPosition(0, 0);
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        viewer->setShowFPS(false);
        viewer->addCoordinateSystem();
        viewer->initCameraParameters();
        viewer->resetCamera();

    }

    ~PCLView(){
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

    std::vector<double> getMinMax(cv::Mat& arr){
        cv::Size size = arr.size();
        double h = size.height;
        double w = size.width;
        std::vector<double> data(2);
        data[0] = h; data[1] = w;
        return data;
    }





    void depth2PCL(const sensor_msgs::ImageConstPtr& msg){
        try{
            depth_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1) -> image;
        

    
            depth_image.convertTo(depth_m, CV_32F, 0.001); //change mm to m
            cv::Size depth_size = depth_avg.size();
            cv::Size rgb_size = rgb_image_.size();
            // double alpha = 0.5;
            // if(!depth_avg.empty()){
            //     depth_avg = alpha * depth_m + (1 - alpha) * depth_avg;
            // }
            // else{
            //     depth_avg = depth_m.clone();
            // }
            depth_avg = depth_m.clone();
            // double minVal;
            // double maxVal;
            // cv::Point minLoc;
            // cv::Point maxLoc;
            // cv::minMaxLoc(depth_avg, &minVal, &maxVal, &minLoc, &maxLoc);
            // std::cout << "Minimum value: " << minVal << " at location: " << minLoc << std::endl;
            // std::cout << "Maximum value: " << maxVal << " at location: " << maxLoc << std::endl;
            int rows = depth_size.height;
            int cols = depth_size.width;
            // std::cout << " H x W (depth)== " << rows << " X "<< cols << std::endl;
            // int r  = rgb_size.height;
            // int c = rgb_size.width;
            // std::cout << " H x W (rgb)== " << r << " X "<< c << std::endl;

            cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointXYZRGB point;


            for(int v = 0; v < rows; ++v){
                for(int u = 0; u < cols; ++u){
                    float z = depth_avg.at<float>(v, u);
                    if(z <= 0.1 || z > max_dist_clip){
                        continue;
                    }
                    pcl::PointXYZRGB point;
                    point.x = (u - cx) * z / fx;
                    point.y = (v - cy) * z / fy;
                    point.z = z;
                    point.b = rgb_image_.at<cv::Vec3b>(v, u)[0];
                    point.g = rgb_image_.at<cv::Vec3b>(v, u)[1];
                    point.r = rgb_image_.at<cv::Vec3b>(v, u)[2];
                    cloud->push_back(point);

                }
            }
            if (!viewer->updatePointCloud(cloud, "depth_cloud")) {

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
                viewer->addPointCloud(cloud, color_handler, "depth_cloud");
            }
            viewer->spinOnce(1);

            




            
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception (Depth) : %s", e.what());
        }
    }








private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;
    cv::Mat rgb_image_, depth_avg, depth_m, depth_image;
    double fx, fy, cx,cy, max_dist_clip;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler;

};


int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_View");
    ros::NodeHandle nh;

    PCLView viewer(nh);
    ros::spin();
    return 0;
}
