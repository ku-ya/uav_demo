#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
using namespace std;

tf::StampedTransform tf_uav;

class Pcl_conversion{
private:
  ros::NodeHandle nh;
  ros::Publisher pcl_pub;
  ros::Subscriber pcl_sub;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  sensor_msgs::PointCloud2 cloud_msg;
  string topicIn;
  string topicOut;
  string frameIn;
  string frameOut;


public:
  bool firstSpin;
  void tf_listen(){
    try{
      listener.lookupTransform(frameIn, frameOut,
                         ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      if(!firstSpin)
        ROS_ERROR("%s",ex.what());
      else
        firstSpin = false;
      ros::Duration(1.0).sleep();
    }
  }
  void pcl_publish(){
    pcl_pub.publish(cloud_msg);
  }
  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input){
    frameIn = input->header.frame_id;// comment if frame_id of input not set
    try{
      pcl_ros::transformPointCloud(frameOut, transform.inverse(), *input, cloud_msg);
      pcl_publish();
    }
    catch (tf::TransformException &ex){
      ROS_WARN("%s\n", ex.what());
    }

  }
  Pcl_conversion(ros::NodeHandle* node):nh(*node){

    // Topic Names
    nh.param("topic_in",  topicIn,  topicIn );
    nh.param("topic_out", topicOut, topicOut);
    ROS_INFO("PCL2 (%s) converted to PCL2 (%s)", topicIn.c_str(), topicOut.c_str());


    // Reference Frames
//    nh.param("frame_in",  frameIn,  frameIn );// uncomment if frame_id of input not set
    nh.param("frame_out", frameOut, frameOut);

    // Subscribe & Publish
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topicOut, 10);
    pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>
        (topicIn, 10, &Pcl_conversion::pclCallback, this);
  }
  virtual ~Pcl_conversion() = default;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_conversion");
  ros::NodeHandle node("~");

  Pcl_conversion pcl_node(&node);
  pcl_node.firstSpin = true;
  ros::spinOnce();

  ros::Rate rate(10.0);
  while (node.ok()){
    pcl_node.tf_listen();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
