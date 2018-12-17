#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class ImageNode{
  public:
  ImageNode();
  ~ImageNode();

  private:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;

  void imgCallback(const sensor_msgs::ImageConstPtr& img_in);

};

ImageNode::ImageNode(): it_(nh_) {
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("rail", 1);

  img_sub_ = it_.subscribe("/this_thorvald/camera1/image_raw", 1, &ImageNode::imgCallback, this);
  img_pub_ = it_.advertise("/rail/detected", 1);
}

ImageNode::~ImageNode(){

}

void ImageNode::imgCallback(const sensor_msgs::ImageConstPtr& img_in){
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImage cv_out;

  // convert ROS image to CV image
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_out.image = cv_ptr->image.clone();

  // perform canny edge detector and hough transform
  cv::Mat canny_image;
  std::vector<cv::Vec4i> lines;
  cv::Canny(cv_out.image, canny_image, 50, 200);
  cv::HoughLinesP(canny_image, lines, 1, CV_PI/180, 50);

  // isolate left and right lines
  std::vector<double> left_phi;
  std::vector<double> right_phi;
  std::vector<cv::Point> left_point;
  std::vector<cv::Point> right_point;
  for( size_t i = 0; i < lines.size(); i++ )
  {
    cv::Vec4i l = lines[i];
    double phi = atan2((l[3]-l[1]), (l[2]-l[0]));
    // remove horizon and horizontal rail
    if (phi < -0.1)
    {
      left_phi.push_back(phi);
      left_point.push_back(cv::Point(l[0],l[1])); left_point.push_back(cv::Point(l[2],l[3]));
      cv::line(cv_out.image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
    }
    else if (phi > 0.1)
    {
      right_phi.push_back(phi);
      right_point.push_back(cv::Point(l[0],l[1])); right_point.push_back(cv::Point(l[2],l[3]));
      cv::line(cv_out.image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
    }
  }


  if(left_point.size() != 0 && right_point.size() != 0)
  {
    // find points with highest and lowest y in the left lines
    cv::Point left_min_y = left_point[0];
    cv::Point left_max_y = left_point[0];
    for(size_t i=0; i<left_point.size(); i++)
    {
      if (left_point[i].y < left_min_y.y){left_min_y = left_point[i];}
      if (left_point[i].y > left_max_y.y){left_max_y = left_point[i];}
    }
    // find points with highest and lowest y in the right lines
    cv::Point right_min_y = right_point[0];
    cv::Point right_max_y = right_point[0];
    for(size_t i=0; i<right_point.size(); i++)
    {
      if (right_point[i].y < right_min_y.y){right_min_y = right_point[i];}
      if (right_point[i].y > right_max_y.y){right_max_y = right_point[i];}
    }
    // draw middle line
    cv::Point upper_point;
    cv::Point lower_point;
    upper_point.x = (left_max_y.x +  right_max_y.x) / 2;
    upper_point.y = (left_max_y.y +  right_max_y.y) / 2;
    lower_point.x = (left_min_y.x +  right_min_y.x) / 2;
    lower_point.y = (left_min_y.y +  right_min_y.y) / 2;
    cv::line(cv_out.image, upper_point, lower_point, cv::Scalar(0,0,255), 1, CV_AA);
  }

  // find average phi
  //double left_phi_mean = 0;
  //double right_phi_mean = 0;
  //for (size_t i = 0; i<left_phi.size(); i++) { left_phi_mean += left_phi[i] / left_phi.size(); }
  //for (size_t i = 0; i<right_phi.size(); i++) { right_phi_mean += right_phi[i] / right_phi.size(); }
  //double phi_mean = (left_phi_mean + right_phi_mean) / 2 + CV_PI/2;
  //cv::line(cv_out.image, cv::Point(upper_point.x, upper_point.y), cv::Point(upper_point.x - 200*cos(phi_mean), upper_point.y - 200*sin(phi_mean)), cv::Scalar(0,0,255), 1, CV_AA);

  // publish image on /rail/detected
  cv_out.encoding = sensor_msgs::image_encodings::BGR8;
  img_pub_.publish(cv_out.toImageMsg());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_detection");
  ImageNode rail_detection;
  ros::spin();
  return 0;
}
