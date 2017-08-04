#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
using namespace std;

class My_Filter {
public:
  ros::NodeHandle node_;
  My_Filter();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
private:
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  string topicIn;
  string topicOut;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){

  // Topic Names
  ros::NodeHandle privateNH("~");
  privateNH.param("topic_in",  topicIn,  topicIn );
  privateNH.param("topic_out", topicOut, topicOut);
  ROS_INFO("Laser (%s) converted to PCL2 (%s)", topicIn.c_str(), topicOut.c_str());

  // Subscribe & Publish
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> (topicIn, 100, &My_Filter::scanCallback, this);
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (topicOut, 100, false);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  string frameID = scan->header.frame_id;
  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud(frameID, *scan, cloud, tfListener_);
  point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_to_pcl2");

  My_Filter filter;

  ros::spin();

  return 0;
}
