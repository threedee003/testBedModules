#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class RealSenseViewer
{
public:
    RealSenseViewer(ros::NodeHandle& nh)
        : it_(nh)
    {
        rgb_sub_ = it_.subscribe("/camera/color/image_raw", 1, &RealSenseViewer::rgbCallback, this);
        depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &RealSenseViewer::depthCallback, this);

        cv::namedWindow("RGB Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

        ROS_INFO("RealSense Viewer initialized. Listening to RGB + Depth topics...");
    }

    ~RealSenseViewer()
    {
        cv::destroyAllWindows();
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            rgb_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
            cv::imshow("RGB Image", rgb_image_);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (RGB): %s", e.what());
        }
    }

      void depthCallback(const sensor_msgs::ImageConstPtr& msg)
      {
      try {
            cv::Mat depth_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            cv::Mat depth_m;
            depth_image.convertTo(depth_m, CV_32F, 0.001); // mm -> m

            // Clip far distances
            double max_vis_m = 3.0;
            depth_m = cv::min(depth_m, max_vis_m);

            // Smooth depth temporally
            double alpha = 0.5;
            if (!depth_avg.empty())
                  depth_avg = alpha * depth_m + (1 - alpha) * depth_avg;
            else
                  depth_avg = depth_m.clone();

            // Convert to 8-bit for color map
            cv::Mat depth_vis;
            depth_avg.convertTo(depth_vis, CV_8U, 255.0 / max_vis_m);

            cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_TURBO);

            cv::imshow("Depth Image", depth_vis);
            cv::waitKey(1);
      }
      catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (Depth): %s", e.what());
      }
      }



private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_sub_;
    image_transport::Subscriber depth_sub_;
    cv::Mat rgb_image_;
    cv::Mat depth_avg;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_viewer");
    ros::NodeHandle nh;

    RealSenseViewer viewer(nh);
    ros::spin();

    return 0;
}
