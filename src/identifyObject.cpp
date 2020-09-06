#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
    ImageConverter(): it_(nh_)
    {
      image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::chatterCallback, this);
      image_pub_ = it_.advertise("/identify_object", 1); 
    }

    ~ImageConverter()
    {

    }

    void chatterCallback(const sensor_msgs::Image::ConstPtr& inputImage)
    {
      //ROS_INFO("ITS WORKING!!");
      cv_bridge::CvImagePtr RGB_image(new cv_bridge::CvImage());
      cv_bridge::CvImagePtr HSV_image(new cv_bridge::CvImage());
      cv_bridge::CvImagePtr threshold_HSV(new cv_bridge::CvImage());

      cv_bridge::CvImage outputImage;
      outputImage.header = inputImage->header;
      outputImage.encoding = "bgr8";//sensor_msgs::image_encodings::RGB8;

      try
      {
        //Convert from ROS msg to OpenCV Mat
        RGB_image = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      //Convert from RGB to HSV
      cvtColor(RGB_image->image, HSV_image->image, cv::COLOR_BGR2HSV);

      //define range
      int low_H = 25;//0;
      int low_S = 100;//25;
      int low_V = 50;//225;

      int high_H = 35;//180;
      int high_S = 255;//40;
      int high_V = 255;//255;

      inRange(HSV_image->image, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), threshold_HSV->image);
      //cvtColor(threshold_HSV->image, outputImage->image, cv::COLOR_HSV2RGB, 4);
      outputImage.image = threshold_HSV->image;
      image_pub_.publish(outputImage.toImageMsg());
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_Identifier");

  ImageConverter ic;

  ros::spin();
  return 0;
}
