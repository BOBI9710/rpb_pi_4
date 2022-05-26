#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Range.h>

#include <iostream>
#include <iomanip>
#include <math.h>

using namespace cv;
using namespace std;

//set variable for drone pose
int x,y,x_s,y_s;
int d;
int iter =0;
int mode =0;

void rangeCallback(const sensor_msgs::Range::ConstPtr& ran)
{
  float Range = 0;
  Range = ran->range;
  
  Mat range_img;
  range_img = Mat::zeros(Size(360,90),CV_8UC1);

  char ranges[30];
  sprintf(ranges, "tfmini = %.3f [m]",Range);

  putText(range_img, ranges , Point(20,50) ,1,2, Scalar(255,0,0), 2,8);

  imshow("tfmini", range_img);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle nh;

  try
  {
    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    //flip
    Mat img_flip;
    flip(img, img_flip, 0); 
   
    //gray
    Mat gray_img;
    cvtColor(img_flip, gray_img, CV_BGR2GRAY);
    
    //blur
    Mat blur_img;
    GaussianBlur(gray_img, blur_img, Size(9,9), 0);

    //Canny
    Mat canny_img;
    Canny(blur_img, canny_img, 50, 127);

    //circle detection
    Mat img_houghC;
    img_flip.copyTo(img_houghC);
	
    vector<Vec3f> circles;
    HoughCircles(blur_img, circles, CV_HOUGH_GRADIENT, 1, 100, 135, 70, 0, 0);

    for (size_t i = 0; i < circles.size(); i++) 
	{  
          // show circle
            Vec3d c = circles[i];
	    Point center(c[0], c[1]);
	    int radius = c[2];

	    circle(img_houghC, center, radius, Scalar (255, 0, 0), 2);
	    circle(img_houghC, center, 2, Scalar (255, 0, 0), 3);
          
          // detect one circle
       
         // y error data
   	   if (fabs(c[0] - 320.0) < 15){
    		y_s = 0; // error under 15
  	    }
   	   else { 
    	  	y_s = 1; // error over 15 
    	    }
       
         // x error data
   	   if (fabs(c[1] - 240.0) < 15){
    	  	x_s = 0; // error under 15
    	    }
    	   else { 
       		x_s = 1; // error over 15
            }

         //mode setting
           if(x_s == 0 && y_s == 0){ // x_s =0 and y_s =0
             mode = 1; 
             }
           else{
             mode = -1;
             }
                  
          char status_mode[30];
   
           if(mode == 1){
            sprintf(status_mode, "ON CENTER");
            
            char Radius[30];
            sprintf(Radius, "R = %d [pixels]", radius);
            putText(img_houghC, Radius , Point(170,110) ,1,2, Scalar(0,255,0), 2,8);

            // range compute with pixels 
            
            //float R = 121.5;
            float R = 250;
            const double PI = 3.1415926;
            float alpha = R / (radius + 1) ; 
            float xr =  240 * alpha / tan(24.4*(PI/180));

            char XR[30];
            sprintf(XR , "Distance = %.3f [m]", xr * 0.001);
            putText(img_houghC, XR , Point(170,140) ,1,2, Scalar(0,255,0), 2,8);

            circle(img_houghC, center, radius, Scalar (0, 255, 0), 3);
	    circle(img_houghC, center, 2, Scalar (0, 255, 0), 3);
 
            }
            else if(mode == -1){
             sprintf(status_mode, "OFF CENTER");
            } 

        //show variable status at cam
        putText(img_houghC, status_mode , Point(220,430) ,1,2, Scalar(255,0,255), 2,8);
       }

     // nothing detected
     if(circles.size() == 0){ 
        
        char status_mode[30];
        sprintf(status_mode, "NO CIRCLE DETECTED");
        putText(img_houghC, status_mode , Point(180,430) ,1,2, Scalar(255,0,0), 2,8);
     }

    //show camera center 
    circle(img_houghC, Point(320, 240),15, Scalar (255, 0, 255),2 );
    line(img_houghC, Point(310,240), Point(330,240), Scalar(255,0,255), 2, 8);
    line(img_houghC, Point(320,230), Point(320,250), Scalar(255,0,255), 2, 8);
   
    imshow("view", img_houghC);
   //imshow("canny_view", canny_img);
   //imshow("blur_view", blur_img);
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_measurement_node");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::namedWindow("tfmini");
  //cv::namedWindow("canny_view");
  //cv::namedWindow("blur_view");

  ros::Rate loop_rate(20);
 
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/raspicam_node/image_raw", 1, imageCallback);

  ros::Subscriber sub_range = nh.subscribe("/tfmini_ros_node/TFmini", 30, rangeCallback);
  
  ros::spin();
  cv::destroyWindow("view");
  cv::destroyWindow("tfmini");
}
