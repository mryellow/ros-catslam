#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <bow_extract/cvKeypointVec.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class BoWExtractor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub_;
  unsigned int image_index;
  std::string file_format;
  
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  
public:
  BoWExtractor()
    : it_(nh_)
  {
    pub_ = nh_.advertise<bow_extract::cvKeypointVec>("bowKeypoints",1000);

    std::string input = "in", transport = "raw";
    ros::param::get("~in",input);
    ros::param::get("~image_transport",transport);
    file_format = "%04d.yml";
    ros::param::get("~file_format",file_format);
    ROS_INFO("Subscribing to %s using transport %s, output in %s",input.c_str(),transport.c_str(), file_format.c_str()); 

    image_sub_ = it_.subscribe(input.c_str(), 1, &BoWExtractor::imageCb, this, transport);

    cv::namedWindow(WINDOW);

    // SURF Method

    int minHessian = 1000;
    detector = new cv::SurfFeatureDetector(minHessian);
    extractor = new cv::SurfDescriptorExtractor();

    image_index = 0;

    // ORB Method

    /*
    cv::OrbFeatureDetector detector;
    cv::OrbDescriptorExtractor extractor;
    cv::BruteForceMatcher<Hamming> matcher;
    */

    ROS_INFO("Added detector, extractor, matcher");

  }

  ~BoWExtractor()
  {
    cv::destroyWindow(WINDOW);
    //delete &detector;
    //delete &extractor;
    //delete &matcher;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO("Received %d by %d image, depth %d, channels %d", cv_ptr->image.cols,cv_ptr->image.rows, cv_ptr->image.depth(), cv_ptr->image.channels());

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector->detect(cv_ptr->image,keypoints);
    extractor->compute(cv_ptr->image,keypoints,descriptors);
    
    // output to message

    bow_extract::cvKeypointVec cvKeyVec;
    for (unsigned int i = 0; i < keypoints.size(); i++) {

      bow_extract::cvKeypoint cvKey;
      cvKey.x = keypoints[i].pt.x;
      cvKey.y = keypoints[i].pt.y;
      cvKey.size = keypoints[i].size;
      cvKey.angle = keypoints[i].angle;
      cvKey.response = keypoints[i].response;
      cvKey.octave = keypoints[i].octave;
      cvKey.class_id = 0;
      cvKeyVec.keypoints.push_back(cvKey);
    }

    // save to file

    char buffer[100];
    sprintf(buffer,file_format.c_str(),image_index);
    cv::FileStorage fs(buffer, cv::FileStorage::WRITE);
    fs << "descriptors" << descriptors;
    fs.release();
    image_index++;

    // display in window

    cv::Mat img_keypoints;
    drawKeypoints(cv_ptr->image,keypoints,img_keypoints,cv::Scalar(255,0,0),cv::DrawMatchesFlags::DEFAULT);

    cv::imshow(WINDOW, img_keypoints);
    cv::waitKey(3);


    pub_.publish(cvKeyVec);
    
    descriptors.release();
    img_keypoints.release();

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bow_extract");
  BoWExtractor be;
  ros::spin();
  return 0;
}
