#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageUnwrapper
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat map_x,map_y;
  
public:
  ImageUnwrapper()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("image_unwrapped", 1);

    std::string input = "in", transport = "raw";
    ros::param::get("~in",input);
    ros::param::get("~image_transport",transport);
    ROS_INFO("Subscribing to %s using transport %s",input.c_str(),transport.c_str()); 

    image_sub_ = it_.subscribe(input.c_str(), 1, &ImageUnwrapper::imageCb, this, transport);

    //cv::namedWindow(WINDOW);
    
    std::string map_path;
    if (!ros::param::get("~map_path",map_path)) {
      ROS_ERROR("Error: Did not specify remap file path");
    }

    ROS_INFO("Using map file at %s",map_path.c_str());    
	
    cv::FileStorage mapFiles(map_path,cv::FileStorage::READ);
    mapFiles["map_x"] >> map_x;
    mapFiles["map_y"] >> map_y;
    mapFiles.release();

    ROS_INFO("Map for %d x %d image",map_x.cols,map_x.rows);
    
  }

  ~ImageUnwrapper()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat grayImage;
    grayImage.create(cv_ptr->image.size(),CV_8UC1);
    cv::cvtColor(cv_ptr->image,grayImage,CV_BGR2GRAY);

    cv::Mat unwrapImage;
    unwrapImage.create(map_x.size(),grayImage.type());

    cv::remap(grayImage,unwrapImage,map_x,map_y,CV_INTER_CUBIC,cv::BORDER_TRANSPARENT,cv::Scalar(0,0,0));

    //ROS_INFO("Unwrapped %d x %d image to %d x %d",grayImage.cols,grayImage.rows,unwrapImage.cols,unwrapImage.rows);

    //cv::imshow(WINDOW, unwrapImage);
    //cv::waitKey(3);

    cv_ptr->encoding = enc::MONO8;
    cv_ptr->image = unwrapImage;

    image_pub_.publish(cv_ptr->toImageMsg());

    grayImage.release();
    unwrapImage.release();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_unwrapper");
  ImageUnwrapper iw;
  ros::spin();
  return 0;
}
