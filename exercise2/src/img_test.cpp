#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>

image_transport::Publisher image_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    
    sensor_msgs::CameraInfoPtr camera = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());
    camera->D.push_back(0.186463);
    camera->D.push_back(-0.367619);
    camera->D.push_back(-0.004393);
    camera->D.push_back(0.003571);
    camera->D.push_back(0.00);

    camera->K[0] = 518.230686; 
    camera->K[1] = 0.000000;
    camera->K[2] = 320.132471;
    camera->K[3] = 0.000000;
    camera->K[4] = 517.464309;
    camera->K[5] = 255.336121;
    camera->K[6] = 0.000000;
    camera->K[7] = 0.000000;
    camera->K[8] = 1.000000;

    camera->height = 480;
    camera->width  = 640;
    
    camera->distortion_model = 	sensor_msgs::distortion_models::PLUMB_BOB;

    camera->P[0] = 527.235474; 	
    camera->P[1] = 0.000000;	
    camera->P[2] = 321.808769;	
    camera->P[3] = 0.000000;	
    camera->P[4] = 0.000000;	
    camera->P[5] = 529.605774;	
    camera->P[6] = 253.378534;	
    camera->P[7] = 0.000000;	
    camera->P[8] = 0.000000;	
    camera->P[9] = 0.000000;	
    camera->P[10] = 1.000000;	
    camera->P[11] = 0.000000;

    camera->R[0]= 1.000000 ;    	
    camera->R[1]= 0.000000;    	
    camera->R[2]= 0.000000;    	
    camera->R[3]= 0.000000;    	
    camera->R[4]= 1.000000;    	
    camera->R[5]= 0.000000;    	
    camera->R[6]= 0.000000;    	
    camera->R[7]= 0.000000;    	
    camera->R[8]= 1.000000;    	
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera);
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat rectified_img = cv::Mat( cv::Mat::ones(img.rows,img.cols,CV_8UC3));	
    cam_model.rectifyImage(img,rectified_img);
    //ROS_INFO("%f\n", camera_info->K[0]);
    cv::imshow("view", img);
    cv::waitKey(30);
    cv::imshow("undistorted_view", rectified_img);
    cv::waitKey(30);

    ros::Time time = ros::Time::now();	
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = rectified_img;
    
    sensor_msgs::Image undist_sensor_msg;
    cvi.toImageMsg(undist_sensor_msg);
    image_pub.publish(undist_sensor_msg);


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view",cv::WINDOW_NORMAL);
  cv::startWindowThread();

  cv::namedWindow("undistorted_view",cv::WINDOW_NORMAL);
  cv::startWindowThread();


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
  image_pub = it.advertise("/undistorted", 1);
  ros::spin();
  cv::destroyWindow("view");
}

