// Copyright 2022. Jaeun Kim (763k357@gmail.com) and Kanghee Kim(kim.kanghee@gmail.com) all rights reserved.
// added by ICHTHUS, Jaeeun Kim on 20221130
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/opencv.hpp>
#include <chrono>
#include "cv_bridge/cv_bridge.h"

#include <sys/timerfd.h>
#include <unistd.h>
using namespace std::chrono_literals; // NOLINT
namespace ichthus_camera_driver
{

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::CameraInfo;

class IchthusCameraDriverNode : public rclcpp::Node
{
    private:
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    rclcpp::Publisher<Image>::SharedPtr rect_pub_;
    rclcpp::Publisher<CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<cv::VideoCapture> cap_ptr_;

    Image::SharedPtr img_;
    Image::SharedPtr rect_img_;
    CompressedImage::SharedPtr compressed_img_;
    CameraInfo::SharedPtr camera_info_;
    int timerfd_;

    cv::Mat map1_, map2_;


    public:
        IchthusCameraDriverNode(const rclcpp::NodeOptions & options) : Node("ichthus_camera_driver_node", options)
        {  
            image_pub_ = this->create_publisher<Image>("/left_raw", rclcpp::SensorDataQoS().keep_last(1));
            rect_pub_ = this->create_publisher<Image>("/left_rect", rclcpp::SensorDataQoS().keep_last(1));
            compressed_image_pub_ = this->create_publisher<CompressedImage>("/left_comp", rclcpp::SensorDataQoS().keep_last(1));
            camera_info_pub_ = this->create_publisher<CameraInfo>("/left_info", rclcpp::SensorDataQoS().keep_last(1));
            // timer_ = this->create_wall_timer(100ms, std::bind(&IchthusCameraDriverNode::timerCallback, this));
            
            img_ = Image::SharedPtr(new Image());
            rect_img_ = Image::SharedPtr(new Image());
            compressed_img_ = CompressedImage::SharedPtr(new CompressedImage());
            camera_info_ = CameraInfo::SharedPtr(new CameraInfo());
            
            

            
            int capture_width = this->declare_parameter("width", 960); 
            int capture_height = this->declare_parameter("height", 640); 
            int rate = this->declare_parameter("rate", 10);
            int display_width = capture_width;
            int display_height = capture_height;
            int framerate = 30;
            int flip_method = 0;

            /////////// Intrinsics Matrix
          
            camera_info_->distortion_model = "plumb_bob";
            camera_info_->k[0] = 1005.196;
            camera_info_->k[1] = 0;
            camera_info_->k[2] = 492.249;
            camera_info_->k[3] = 0;
            camera_info_->k[4] = 1074.321;
            camera_info_->k[5] = 326.746;
            camera_info_->k[6] = 0;
            camera_info_->k[7] = 0;
            camera_info_->k[8] = 1;

             // For_ monocular camera, rotation matrix is identity matrix
            camera_info_->r[0] = 1;
            camera_info_->r[1] = 0;
            camera_info_->r[2] = 0;
            camera_info_->r[3] = 0;
            camera_info_->r[4] = 1;
            camera_info_->r[5] = 0;
            camera_info_->r[6] = 0;
            camera_info_->r[7] = 0;
            camera_info_->r[8] = 1;

             
            camera_info_->d.push_back( -0.6012);    
            camera_info_->d.push_back(  0.3797);
            camera_info_->d.push_back( 0.001);
            camera_info_->d.push_back( -0.004508);
            camera_info_->d.push_back( 0.0);
            camera_info_->binning_x = 1;
            camera_info_->binning_y = 1;
            camera_info_->roi.height = capture_height;
            camera_info_->roi.width = capture_width;
            camera_info_->height = capture_height;
            camera_info_->width = capture_width;
            
            cv::Size imageSize=cv::Size(capture_width, capture_height);  
            cv::Mat cameraMatrix(3, 3, CV_32FC1);
            for (int i = 0; i<3; i++){
              for (int j = 0; j < 3; j++)
                cameraMatrix.at<float>(i,j) = camera_info_->k[i*3 + j];
            }
            cv::Mat distCoeffs(1,5, CV_32FC1);
            for (int i = 0; i< 5; i++)
              distCoeffs.at<float>(0,i) = camera_info_->d[i];

            /////////////    

            initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_32FC1, map1_, map2_);

            std::string pipeline = gstreamer_pipeline(capture_width,
                                                    capture_height,
                                                    display_width,
                                                    display_height,
                                                    framerate,
                                                    flip_method);
            std::cout << "Using pipeline: \n\t" << pipeline << "\n";

            cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened())
            {
                std::cout << "Failed to open camera." << std::endl;
                exit(0);
            }

            cv::Mat img;
            cv::Mat rect_img;
            cv_bridge::CvImage cv_img;
            cv_bridge::CvImage cv_img_rect;
            std::string fid = "cam1";
            struct itimerspec timeout;
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            timeout.it_value = ts;
            timeout.it_interval.tv_sec = 0;
            timeout.it_interval.tv_nsec = (1000/rate) * 1e6; // rate(100 ms-> 10Hz)
            RCLCPP_INFO(get_logger(), "rate = %dHz", rate);
            RCLCPP_INFO(get_logger(), "Width x Height = %d x %d ", capture_width, capture_height);

            if( (timerfd_ = timerfd_create(CLOCK_REALTIME, 0)) <= 0 ){
                std::cout << "timerfd create failed..." << std::endl;
            }
            if (timerfd_settime(timerfd_, TFD_TIMER_ABSTIME, &timeout, NULL) != 0){
                std::cout << "timerfd_settime fail" << std::endl;
            }
            while (rclcpp::ok())
            {
                auto start = rclcpp::Clock().now().nanoseconds();
                if (!cap.read(img))
                {
                    std::cout << "##### Capture read error" << std::endl;
                    break;
                }
                
                auto stamp = this->now();
                // std::cout<<"diff1 : "<<(rclcpp::Clock().now().nanoseconds() - start)*1.0e-6<<std::endl;
                
                // Create Raw Image
                cv_img.image = img;
                cv_img.encoding = "rgb8";
                cv_img.toImageMsg(*(img_));
                
                
                std::vector<int> params;
                params.resize(3,0);
                params[0] = cv::IMWRITE_JPEG_QUALITY;
                params[1] = 95;
                compressed_img_->format = "rgb8; jpeg compressed ";
                
                // // Undistort Image
                remap(img, rect_img, map1_, map2_, cv::INTER_LINEAR);

                cv::imencode(".jpg", rect_img, compressed_img_->data, params);
                cv_img_rect.image = rect_img;
                cv_img_rect.encoding = "rgb8";
                cv_img_rect.toImageMsg(*(rect_img_));
                

                compressed_img_->header.stamp = camera_info_->header.stamp = img_->header.stamp = rect_img_->header.stamp  = stamp;
                compressed_img_->header.frame_id = camera_info_->header.frame_id = img_->header.frame_id = rect_img_->header.frame_id = fid; 

                image_pub_->publish(*img_);
                compressed_image_pub_->publish(*compressed_img_);
                rect_pub_->publish(*rect_img_);
                camera_info_pub_->publish(*camera_info_);
                
                unsigned long long missed;
                if(read(timerfd_, &missed, sizeof(missed)) < 0)
                    std::cout << "timer read error" << std::endl;
                // std::cout<<"while loop  : "<<(rclcpp::Clock().now().nanoseconds() - start)*1.0e-6<<std::endl;
                struct timespec a;
                clock_gettime(CLOCK_REALTIME, &a);
                // RCLCPP_INFO(get_logger(), "Curr Time: %ld.%0ld", a.tv_sec, a.tv_nsec);
            }

            cap.release();
        }

        ~IchthusCameraDriverNode()
        {
        }
        
        std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
            return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
            std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
            std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_camera_driver::IchthusCameraDriverNode)
