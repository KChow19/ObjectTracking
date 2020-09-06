#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo.hpp>

static const std::string OPENCV_WINDOW = "Image Window";
cv::RNG rng(12345);

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
      image_pub_ = it_.advertise("/filtered_image", 1);
      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void chatterCallback(const sensor_msgs::Image::ConstPtr& inputImage)
    {
      	cv_bridge::CvImagePtr BGR_image(new cv_bridge::CvImage());
        cv_bridge::CvImagePtr HSV_image(new cv_bridge::CvImage());
        cv_bridge::CvImagePtr threshold_HSV(new cv_bridge::CvImage());
      
        try
        {
          BGR_image = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        cv::Mat BGR_image_filtered;
        bilateralFilter(BGR_image->image, BGR_image_filtered, 9, 75, 75);
        
        //Convert from RGB to HSV
        cvtColor(BGR_image_filtered, HSV_image->image, cv::COLOR_BGR2HSV);
        
        //define range
        int low_H = 15;
        int low_S = 50;
        int low_V = 50;

        int high_H = 45;
        int high_S = 255;
        int high_V = 255;
        
        inRange(HSV_image->image, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), threshold_HSV->image);
        //ROS_INFO("channels: %d", threshold_HSV->image.channels());
        
        cv::Mat greyscale;
        cv::Mat hsv_channels[3];
        cv::split(threshold_HSV->image, hsv_channels);
        greyscale = hsv_channels[0];

        //Filtering
        /*
        cv::Mat filtered_greyscale;
        cv::fastNlMeansDenoising(greyscale, filtered_greyscale, 7, 21, 20);
        */
                
        cv::Mat dst, detected_edges;
        Canny(greyscale, detected_edges, 200, 300, 3); //TODO: change to filtered_greyscale
        dst = cv::Scalar::all(0);
        
        greyscale.copyTo(dst, detected_edges); //TODO: change to filtered_greyscale
        
        std::vector<std::vector<cv::Point>> contours;
        findContours(dst, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<std::vector<cv::Point> > contours_poly(contours.size());
        std::vector<cv::Rect> boundRect(contours.size());
        
        double MinAreaContour = -1;
        double MaxAreaContour = 99999999;
        
        for (size_t i = 0; i < contours.size(); i++)
        {
          double AreaContour = contourArea(contours[i]);
          if(AreaContour < MaxAreaContour && AreaContour > MinAreaContour)
          {
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(contours_poly[i]);
          }
        }
        
        cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);
        
        for( size_t i = 0; i< contours.size(); i++ )
        {
          cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
          rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
        }

        cv::Mat RGB_detected_edges(detected_edges.size(), CV_8UC3);
        cvtColor(detected_edges, RGB_detected_edges, CV_GRAY2RGB);

        cv::Mat output;
        //addWeighted(drawing, 0.5, BGR_image->image, 0.5, 0.0, output); //Overlayed with input image
        addWeighted(drawing, 0.5, RGB_detected_edges, 0.5, 0.0, output); //Overlayed with contours image
        
        //cv::imshow(OPENCV_WINDOW, threshold_HSV->image); //Greyscale image
        cv::imshow(OPENCV_WINDOW, output); //Final detection image

        //cv::imshow(OPENCV_WINDOW, detected_edges); //Contours image
        cv::waitKey(3);
        
        //image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_Filter");

  ImageConverter ic;

  ros::spin();
  return 0;
}
