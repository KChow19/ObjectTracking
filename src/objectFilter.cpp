#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo.hpp>

static const std::string OPENCV_WINDOW = "Image Window";
cv::RNG rng(12345);

//define range
int low_H = 0; //14
int low_S = 0; //142
int low_V = 0; //115

int high_H = 255; //27
int high_S = 255; //255
int high_V = 255; //255

int MinAreaContour = 4000;
int MaxAreaContour = 4000;

int x_position = 0;
int y_position = 0;

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
        bilateralFilter(BGR_image->image, BGR_image_filtered, 15, 150, 150);
        
        //Convert from RGB to HSV
        cvtColor(BGR_image_filtered, HSV_image->image, cv::COLOR_BGR2HSV);
        
        inRange(HSV_image->image, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), threshold_HSV->image);
        //ROS_INFO("channels: %d", threshold_HSV->image.channels());
        
        cv::Mat greyscale;
        cv::Mat hsv_channels[3];
        cv::split(threshold_HSV->image, hsv_channels);
        greyscale = hsv_channels[0];

        //Filtering
        cv::Mat erosion_greyscale;
        int erosion_size = 10; //Max value is 21 TODO: Tune erosion value
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*erosion_size+1, 2*erosion_size+1), cv::Point(erosion_size, erosion_size));
        erode(greyscale, erosion_greyscale,element);

        cv::Mat filtered_greyscale;
        GaussianBlur(erosion_greyscale, filtered_greyscale, cv::Size(3,3), 21, 21, cv::BORDER_DEFAULT);

        cv::Mat dst, detected_edges;
        Canny(filtered_greyscale, detected_edges, 200, 300, 3); //TODO: change to filtered_greyscale
        dst = cv::Scalar::all(0);
        
        filtered_greyscale.copyTo(dst, detected_edges); //TODO: change to filtered_greyscale
        
        std::vector<std::vector<cv::Point>> contours;
        findContours(dst, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        
        std::vector<std::vector<cv::Point> > contours_poly(contours.size());
        std::vector<cv::Rect> boundRect(contours.size());
        
        for (size_t i = 0; i < contours.size(); i++)
        {
          //double AreaContour = contourArea(contours[i]);
          //if(AreaContour < MaxAreaContour && AreaContour > MinAreaContour)
          {
            //approxPolyDP(contours[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(contours[i]);
          }
        }
        
        cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);
        
        for( size_t i = 0; i< contours.size(); i++ )
        {
          cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
          rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
        }

        cv::Mat RGB_detected_edges(detected_edges.size(), CV_8UC3);
        cvtColor(detected_edges, RGB_detected_edges, CV_GRAY2RGB);

        cv::Mat output1, output2;
        addWeighted(drawing, 0.5, BGR_image->image, 0.5, 0.0, output2); //Overlayed with input image
        //addWeighted(drawing, 0.5, RGB_detected_edges, 0.5, 0.0, output); //Overlayed with contours image
        addWeighted(BGR_image->image, 0.5, RGB_detected_edges, 0.5, 0.0, output1); //Overlayed input and contours images
        
        // Trackbars to set thresholds for HSV values
        cv::createTrackbar("Low H", OPENCV_WINDOW, &low_H, 255, low_H_trackbar);
        cv::createTrackbar("High H", OPENCV_WINDOW, &high_H, 255, high_H_trackbar);
        cv::createTrackbar("Low S", OPENCV_WINDOW, &low_S, 255, low_S_trackbar);
        cv::createTrackbar("High S", OPENCV_WINDOW, &high_S, 255, high_S_trackbar);
        cv::createTrackbar("Low V", OPENCV_WINDOW, &low_V, 255, low_V_trackbar);
        cv::createTrackbar("High V", OPENCV_WINDOW, &high_V, 255, high_V_trackbar);
        
        cv::createTrackbar("Low Area", "Final Output", &MinAreaContour, 5000, low_area_trackbar);
        cv::createTrackbar("High Area", "Final Output", &MaxAreaContour, 10000, high_area_trackbar);
        
        //Determining the x and y location of the object
        double areaContour = 0;
        if(!boundRect.empty())
        {
          for(int i = 0; i < boundRect.size(); i++)
          {
            areaContour = contourArea(contours[i]);
            //if(boundRect[i].x != 0 && boundRect[i].y != 0)
            if(areaContour < MaxAreaContour && areaContour > MinAreaContour)
            {
              x_position = (x_position + (boundRect[i].x + boundRect[i].width /2))/2;
              y_position = (y_position + (boundRect[i].y + boundRect[i].height /2))/2;
              //x_position = (x_position + boundRect[i].x)/2;
              //y_position = (y_position + boundRect[i].y)/2;
            }
            circle(output2, cv::Point(x_position, y_position), 10, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);
          }
        }

        putText(output2, "x: " + std::to_string(x_position), cv::Point2f(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 255));
        putText(output2, "y: " + std::to_string(y_position), cv::Point2f(10, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 255));
        putText(output2, "Area: " + std::to_string(areaContour), cv::Point2f(10, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 255));

        //cv::imshow(OPENCV_WINDOW, BGR_image->image); //Input image
        //cv::imshow(OPENCV_WINDOW, BGR_image_filtered); //Filtered input image
        //cv::imshow(OPENCV_WINDOW, threshold_HSV->image); //Greyscale image
        //cv::imshow(OPENCV_WINDOW, filtered_greyscale); //Filtered Greyscale image
        //cv::imshow(OPENCV_WINDOW, detected_edges); //Canny edge image
        cv::imshow(OPENCV_WINDOW, output1); //Final detection image
        cv::imshow("Final Output", output2);
        if(cv::waitKey(3) == 's')
        {
          //imwrite("sample.png", BGR_image->image);
          imwrite("filtered_output.png", output1);
        }
        //image_pub_.publish(cv_ptr->toImageMsg());
    }
    
    static void low_H_trackbar(int, void *)
    {
        low_H = cv::min(high_H-1, low_H);
        cv::setTrackbarPos("Low H", OPENCV_WINDOW, low_H);
    }
    static void high_H_trackbar(int, void *)
    {
        high_H = cv::max(high_H, low_H+1);
        cv::setTrackbarPos("High H", OPENCV_WINDOW, high_H);
    }
    static void low_S_trackbar(int, void *)
    {
        low_S = cv::min(high_S-1, low_S);
        cv::setTrackbarPos("Low S", OPENCV_WINDOW, low_S);
    }
    static void high_S_trackbar(int, void *)
    {
        high_S = cv::max(high_S, low_S+1);
        cv::setTrackbarPos("High S", OPENCV_WINDOW, high_S);
    }
    static void low_V_trackbar(int, void *)
    {
        low_V = cv::min(high_V-1, low_V);
        cv::setTrackbarPos("Low V", OPENCV_WINDOW, low_V);
    }
    static void high_V_trackbar(int, void *)
    {
        high_V = cv::max(high_V, low_V+1);
        cv::setTrackbarPos("High V", OPENCV_WINDOW, high_V);
    }
    static void low_area_trackbar(int, void *)
    {
        MinAreaContour = cv::min(MaxAreaContour-1, MinAreaContour);
        cv::setTrackbarPos("Low Area", "Final Output", MinAreaContour);
    }
    static void high_area_trackbar(int, void *)
    {
        MaxAreaContour = cv::max(MaxAreaContour, MinAreaContour+1);
        cv::setTrackbarPos("High Area", "Final Output", MaxAreaContour);
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_Filter");

  ImageConverter ic;

  ros::spin();
  return 0;
}
