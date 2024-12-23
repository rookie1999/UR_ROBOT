#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/image_encodings.h"

// 定义了一个静态常量字符串OPENCV_WINDOW，用于在OpenCV窗口中显示图像时的窗口标题
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;  // 处理图像传输
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
    ImageConverter() : it_(nh_) {  // 初始化时调用ImageTransport的构造函数，并传入nodehandle
      // subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::image_callback, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

      // 创建了一个名为OPENCV_WINDOW的OpenCV窗口
      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
      // 销毁名为OPENCV_WINDOW的OpenCV窗口
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg) {
      // 定义了一个cv_bridge::CvImagePtr类型的智能指针cv_ptr，用于存储转换后的OpenCV图像
      cv_bridge::CvImagePtr cv_ptr;
      try {
        // 使用cv_bridge库将ROS图像消息转换为BGR8格式的OpenCV图像，并复制到cv_ptr中
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      // Draw a circle on the video stream
      // 如果图像的行和列都大于60，就在图像上绘制一个红色的圆
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) 
        cv::circle(cv_ptr->image, cv::Point(50, 50), 30, CV_RGB(250, 0, 0), -1);
      
      // Update to the GUI window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Output the modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
