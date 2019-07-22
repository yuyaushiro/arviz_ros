#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


class PubScanInUnity
{
 public:
  PubScanInUnity();
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

 private:
  ros::NodeHandle nh_;

  laser_geometry::LaserProjection projector_;
  tf::TransformListener tf_listener_;

  ros::Subscriber scan_sub_;
  ros::Publisher pc_pub_;
};

PubScanInUnity::PubScanInUnity()
{
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1,
                                                     &PubScanInUnity::scanCallback, this);
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>("unity/scan_pc", 1, false);
}

void PubScanInUnity::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  // if(!tf_listener_.waitForTransform(
  //     "base_scan", "unity",
  //     // scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
  //     ros::Time::now(), ros::Duration(1.0)))
  // {
  //   return;
  // }
  try
  {
    tf_listener_.waitForTransform("base_scan", "unity",
                                  scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                                  ros::Duration(3.0));
    sensor_msgs::PointCloud pc;
    sensor_msgs::PointCloud pc_in_unity;
    // map座標系のLaserScanをunity座標系のPointCloudに変換
    // projector_.transformLaserScanToPointCloud("unity", scan_, pc, tf_listener_);
    projector_.projectLaser(*scan, pc);
    tf_listener_.transformPointCloud("unity", scan->header.stamp, pc, "base_scan", pc_in_unity);
    pc_pub_.publish(pc_in_unity);
    // pc_pub_.publish(pc);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_scan_in_unity");

  PubScanInUnity pub_scan_in_unity;
  ros::spin();

  return 0;
}
