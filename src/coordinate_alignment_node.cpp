#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>


class CoordinateAlignment
{
 public:
  CoordinateAlignment();

  // Subscriber
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  // 座標合わせ実行指示
  void AlignmentCallback(const std_msgs::EmptyConstPtr& data);

 private:
  // ノードハンドラ
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber alignment_sub_;
};

CoordinateAlignment::CoordinateAlignment()
{
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1,
                                                    &CoordinateAlignment::mapCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1,
                                                    &CoordinateAlignment::scanCallback, this);
  alignment_sub_ = nh_.subscribe<std_msgs::Empty>("icp_align", 1,
                                                  &CoordinateAlignment::AlignmentCallback, this);
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_alignment");

  CoordinateAlignment coordinate_alignment;
  ros::spin();

  return 0;
}
