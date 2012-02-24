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
  
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Mat codebook;
  
public:
  BoWExtractor()
    : it_(nh_)
  {
    pub_ = nh_.advertise<bow_extract::cvKeypointVec>("bowKeypoints",1000);

    std::string input = "in", transport = "raw";
    ros::param::get("~in",input);
    ros::param::get("~image_transport",transport);
    ROS_INFO("Subscribing to %s using transport %s",input.c_str(),transport.c_str()); 

    image_sub_ = it_.subscribe(input.c_str(), 1, &BoWExtractor::imageCb, this, transport);

    cv::namedWindow(WINDOW);
    
    std::string codebook_path;
    if (!ros::param::get("~codebook_path",codebook_path)) {
      ROS_ERROR("Error: Did not specify codebook path");
    }

    ROS_INFO("Using codebook file at %s",codebook_path.c_str());    

    cv::FileStorage codebookFile(codebook_path,cv::FileStorage::READ);
    codebookFile["codebook"] >> codebook;
    codebookFile.release();

    ROS_INFO("Codebook with %d words, %d dims loaded",codebook.rows,codebook.cols);

    // SURF Method

    int minHessian = 1000;
    detector = new cv::SurfFeatureDetector(minHessian);
    extractor = new cv::SurfDescriptorExtractor();
    matcher = new cv::FlannBasedMatcher();

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
    std::vector<cv::DMatch> matches;

    detector->detect(cv_ptr->image,keypoints);
    extractor->compute(cv_ptr->image,keypoints,descriptors);
    matcher->match(descriptors,codebook,matches);
    
    // output to message

    bow_extract::cvKeypointVec cvKeyVec;
    for (unsigned int i = 0; i < keypoints.size(); i++) {
      
      keypoints[i].class_id = matches[i].trainIdx;

      bow_extract::cvKeypoint cvKey;
      cvKey.x = keypoints[i].pt.x;
      cvKey.y = keypoints[i].pt.y;
      cvKey.size = keypoints[i].size;
      cvKey.angle = keypoints[i].angle;
      cvKey.response = keypoints[i].response;
      cvKey.octave = keypoints[i].octave;
      //cvKey.class_id = 0;
      cvKey.class_id = keypoints[i].class_id;
      cvKeyVec.keypoints.push_back(cvKey);
    }

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
