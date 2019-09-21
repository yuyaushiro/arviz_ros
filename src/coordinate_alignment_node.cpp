#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


class CoordinateAlignment
{
 public:
  CoordinateAlignment();

  // Subscriber
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  // 座標合わせ実行指示
  void AlignmentCallback(const std_msgs::EmptyConstPtr& data);

  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

 private:
  // ノードハンドラ
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber alignment_sub_;

  ros::Publisher pointcloud_pub_;  // デバッグ用に点群のPub
};

CoordinateAlignment::CoordinateAlignment()
{
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1,
                                                    &CoordinateAlignment::mapCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1,
                                                    &CoordinateAlignment::scanCallback, this);
  alignment_sub_ = nh_.subscribe<std_msgs::Empty>("icp_align", 1,
                                                  &CoordinateAlignment::AlignmentCallback, this);

  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_out", 1, true);
}

void CoordinateAlignment::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
}

void CoordinateAlignment::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
}

void CoordinateAlignment::AlignmentCallback(const std_msgs::EmptyConstPtr& data)
{
}

void CoordinateAlignment::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(*point_cloud, pc2);
  pc2.header.frame_id = "map";
  pointcloud_pub_.publish(pc2);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_alignment");

  CoordinateAlignment coordinate_alignment;
  ros::spin();

  return 0;
}
