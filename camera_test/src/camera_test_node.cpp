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
  ROS_INFO("range = %.3f", ran->range);
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
    HoughCircles(blur_img, circles, CV_HOUGH_GRADIENT, 1, 100, 135, 55, 0, 0);

    for (size_t i = 0; i < circles.size(); i++) 
	{  
          // show circle
            Vec3d c = circles[i];
	    Point center(c[0], c[1]);
	    int radius = c[2];

	    circle(img_houghC, center, radius, Scalar (255, 0, 0), 2);
	    circle(img_houghC, center, 2, Scalar (255, 0, 0), 3);
          
        // detect one circle -> spwan to start
       
         // y error data
   	   if (fabs(c[0] - 320.0)<15){
    		y_s=0; // error under 15 
  	    }
   	   else { 
    	  	y_s=1; // error over 15 
    	    }
        
            y = - c[0] + 320 ; // y error save
       
         // x error data
   	   if (fabs(c[1] - 240.0)<15){
    	  	x_s=0; // error under 15
    	    }
    	   else { 
       		x_s=1; // error over 15
            }
         
            x = - c[1] + 240; // x error save

         //mode setting
           if(x_s ==0 && y_s==0){ // x_s =0 and y_s =0 for certain time -> alt down to 0.9m
             iter = iter +1;
             }
             else{
             iter =0;
             }
       
             if(iter >= 30){
                  mode = -1;            
               }
             else{
                  mode = 1;
             }        

          char status_x[30];
          sprintf(status_x, "x = %d , x_s = %d",x,x_s);
          char status_y[30];
          sprintf(status_y, "y = %d , y_s = %d",y,y_s);
          char status_mode[30];
          sprintf(status_mode, "mode = %d",mode);

        //show variable status at cam
        putText(img_houghC, status_mode , Point(20,430) ,1,1, Scalar(0,0,0), 1,8);
        putText(img_houghC, status_x , Point(20,450) ,1,1, Scalar(0,0,0), 1,8);
        putText(img_houghC, status_y , Point(20,470) ,1,1, Scalar(0,0,0), 1,8);
       }

     // nothing detected -> start -> line tracking
     if(circles.size() == 0){ 
        mode = -2;
        
        char status_mode[30];
        sprintf(status_mode, "mode = %d",mode);
        putText(img_houghC, status_mode , Point(20,430) ,1,1, Scalar(0,0,0), 1,8);
         }

    //show camera center 
    circle(img_houghC, Point(320, 240),15, Scalar (255, 0, 255),2 );
    line(img_houghC, Point(310,240), Point(330,240), Scalar(255,0,255), 2, 8);
    line(img_houghC, Point(320,230), Point(320,250), Scalar(255,0,255), 2, 8);
   
    imshow("view", img_houghC);
   //imshow("canny_view", canny_img);
   imshow("blur_view", blur_img);
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_test_node");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  //cv::namedWindow("canny_view");
  cv::namedWindow("blur_view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/raspicam_node/image_raw", 1, imageCallback);

  ros::Subscriber sub_range = nh.subscribe("/tfmini_ros_node/TFmini", 100, rangeCallback);

  ros::spin();
  cv::destroyWindow("view");
}
