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

public:
    void tf_listen()
    {
        try{
            listener.lookupTransform("/camera_rgb_optical_frame","/world",
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
    }
    void pcl_publish()
    {
        pcl_pub.publish(cloud_msg);
    }
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {

    	try{
    	   pcl_ros::transformPointCloud("/world", transform.inverse(), *input, cloud_msg);
           pcl_publish();
    	}
    	catch (tf::TransformException &ex){
    		ROS_WARN("%s\n", ex.what());
    	}
    }
    Pcl_conversion(ros::NodeHandle* node):nh(*node)
    {
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_pcl", 10);
        pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>
            ("/camera/depth_registered/points", 10, &Pcl_conversion::pclCallback, this);
    }
    virtual ~Pcl_conversion() = default;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pcl_conversion");
  ros::NodeHandle node;
  Pcl_conversion pcl_node(&node);

  ros::Rate rate(10.0);
  while (node.ok()){
    pcl_node.tf_listen();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
