#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
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
  void convertMapIntoPointCloud(const nav_msgs::OccupancyGridConstPtr& occupancy_grid,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

 private:
  // ノードハンドラ
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber alignment_sub_;

  ros::Publisher pointcloud_pub_;  // デバッグ用に点群のPub

  // 点群
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_;
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

  map_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void CoordinateAlignment::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  convertMapIntoPointCloud(map, map_pc_);
  publishPointCloud(map_pc_);
}

void CoordinateAlignment::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
}

void CoordinateAlignment::AlignmentCallback(const std_msgs::EmptyConstPtr& data)
{
}

void CoordinateAlignment::convertMapIntoPointCloud(const nav_msgs::OccupancyGridConstPtr& occupancy_grid,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
   // マップ情報
  std::vector<int8_t> map_image = occupancy_grid->data;
  float map_resolution = occupancy_grid->info.resolution;
  int map_width = occupancy_grid->info.width;
  
  // resizeの為にカウント
  size_t point_num = 0;  // 障害物格子の数
  for (int pixel : map_image)
    if (pixel >= 100) point_num++;

  // 点群情報
  point_cloud->width = point_num;
  point_cloud->height = 1;
  point_cloud->is_dense = false;
  point_cloud->points.resize(point_cloud->width * point_cloud->height);  // 予めresize

  //点群を画素の左下ではなく中心に生成するためのオフセット
  float pixel_offset = map_resolution / 2;

  // 障害物格子から点群を生成
  size_t point_index = 0;  // 生成した点の数
  for (int i = 0; i < std::end(map_image) - std::begin(map_image); i++)
  {
    if (map_image[i] >= 100)
    {
      point_cloud->points[point_index].x = (float)(i % map_width) * map_resolution + pixel_offset;
      point_cloud->points[point_index].y = (float)(i / map_width) * map_resolution + pixel_offset;
      point_cloud->points[point_index].z = 0.0;
      point_index++;
    }
  }

  // map座標系への座標変換
  tf::Transform transform_map;
  tf::poseMsgToTF(occupancy_grid->info.origin, transform_map);
  pcl_ros::transformPointCloud(*point_cloud, *point_cloud, transform_map);
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
