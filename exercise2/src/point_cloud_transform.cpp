#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

image_transport::Publisher image_pub;
ros::Publisher pcl_pub;
tf::TransformListener *listener;

void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    
    cv::Mat rgb_img = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    cv::Mat depth_img = cv_bridge::toCvShare(depth_msg)->image;
    
    //ROS_ERROR("Encoding from '%s'.", cv_bridge::toCvCopy(rgb_msg)->encoding.c_str());
    //cv::Mat depth_img = cv::Mat( cv::Mat::ones(700,700,CV_8UC3));

    //ros::Time time = ros::Time::now();	
    //cv_bridge::CvImage cvi;
    //cvi.header.stamp = time;
    //cvi.header.frame_id = "image";
    //cvi.encoding = "bgr8";
    //cvi.image = rectified_img;
    
    //sensor_msgs::Image undist_sensor_msg;
    //cvi.toImageMsg(undist_sensor_msg);
    //image_pub.publish(undist_sensor_msg);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 output;	

    cloud.width  = rgb_img.cols;
    cloud.height = rgb_img.rows;
    cloud.points.resize(cloud.width * cloud.height);	
    
    float fx = 525.0 ; // focal length x
    float fy = 525.0 ; // focal length y
    float cx = 319.5 ; // optical center x
    float cy = 239.5 ; // optical center y

    float factor = 1.0;

    float X,Y,Z;	
    for(int v=0;v<rgb_img.rows;v++){
    	for(int u=0; u<rgb_img.cols; u++){
         cv::Vec3b color = rgb_img.at<cv::Vec3b>(v,u);
	 float dval = depth_img.at<float>(v, u);
	 //float dval = static_cast<float>(d); 		
	 
	 Z = dval/factor;
	 X = (u - cx) * Z / fx;
 	 Y = (v - cy) * Z / fy;
         int point_index = v * rgb_img.cols + u;
   
	 cloud.points[point_index].x = X;
	 cloud.points[point_index].y = Y;
	 cloud.points[point_index].z = Z;
	 cloud.points[point_index].r = color[2];
	 cloud.points[point_index].g = color[1];
	 cloud.points[point_index].b = color[0];


	}
    }	
    sensor_msgs::PointCloud2 transformed_output;
    //tf::StampedTransform transform;
    try{
      listener->waitForTransform("/world", "/kinect",
                               ros::Time::now(), ros::Duration(3));

     // listener.lookupTransform("/world", "/kinect",
       //                        ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "kinect";
	
    //pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl_ros::transformPointCloud("/world",output,transformed_output,*listener);
     
    //pcl::toROSMsg(transformed_cloud, output);
    //output.header.frame_id = "world";
    

    pcl_pub.publish(transformed_output);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;


  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);

  listener = new tf::TransformListener();
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;

  message_filters::Synchronizer<sync_policy> sync(sync_policy(10), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&imageCallback,_1,_2));
  
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

  ros::spin();
}

