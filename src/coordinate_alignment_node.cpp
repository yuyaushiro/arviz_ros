#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


/**
 * @brief 地図と現実世界の座標を合わせる
 */
class CoordinateAlignment
{
 public:
  /// コンストラクタ
  CoordinateAlignment();

  /// マップのコールバック
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
  /// レーザースキャンのコールバック
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  /// 座標合わせ実行指示のコールバック
  void alignmentCallback(const std_msgs::EmptyConstPtr& data);

 private:
  /// 占有格子地図を点群に変換
  void convertMapIntoPointCloud(const nav_msgs::OccupancyGridConstPtr& occupancy_grid,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  /// レーザースキャンを点群に変換
  void convertScanIntoPointCloud(const sensor_msgs::LaserScanConstPtr& scan,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  /// ICPにより点群の重ね合わせ
  void alignWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_pc,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr alined_pc,
                    Eigen::Matrix4d& transformation_matrix);
  /// 点群をパブリッシュ
  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  /// 初期位置をパブリッシュ
  void publishInitialpose(const tf::Transform transform);
  /// Eigenの行列をTFに変換
  void convertMatrix4dIntoTF(const Eigen::Matrix4d& eigen_matrix,
                             tf::Transform& transform);

 private:
  // ノードハンドラ
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber alignment_sub_;

  // デバッグ用の点群パブリッシャ
  ros::Publisher pointcloud_pub_;
  // ICPで格子地図とスキャン合わせて補正した初期姿勢のパブリッシャ
  ros::Publisher initialpose_pub_;  

  // TF
  tf::TransformListener tf_listner_;

  // 点群
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_;
};


/**
 * @brief コンストラクタ
 */
CoordinateAlignment::CoordinateAlignment()
{
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1,
                                                    &CoordinateAlignment::mapCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1,
                                                    &CoordinateAlignment::scanCallback, this);
  alignment_sub_ = nh_.subscribe<std_msgs::Empty>("align", 1,
                                                  &CoordinateAlignment::alignmentCallback, this);

  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_out", 1, true);
  initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1,
                                                                             false);

  map_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  scan_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}


/**
 * @brief マップのコールバック
 */
void CoordinateAlignment::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  convertMapIntoPointCloud(map, map_pc_);
  // publishPointCloud(map_pc_);
}


/**
 * @brief レーザースキャンのコールバック
 */
void CoordinateAlignment::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  try
  {
    tf_listner_.waitForTransform(scan->header.frame_id, "map",
                                 scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                                 ros::Duration(3.0));
    convertScanIntoPointCloud(scan, scan_pc_);
    // publishPointCloud(scan_pc_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


/**
 * @brief 座標合わせ実行指示のコールバック
 */
void CoordinateAlignment::alignmentCallback(const std_msgs::EmptyConstPtr& data)
{
  try
  {
    tf_listner_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(3.0));

    // mapからbasefoot_printへの変換
    tf::StampedTransform map_T_foot;
    tf_listner_.lookupTransform("map", "base_footprint", ros::Time(0), map_T_foot);

    // 点群同士の重ね合わせを計算
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_pc(new pcl::PointCloud<pcl::PointXYZ>);  // 位置合わせ後のscan点群
    Eigen::Matrix4d alignment_transformation_matrix;  // 位置合わせ行うための座標変換
    alignWithICP(scan_pc_, map_pc_, aligned_scan_pc, alignment_transformation_matrix);

    // Matrix4dをtf::Poseに変換
    tf::Transform alignment_transform;
    convertMatrix4dIntoTF(alignment_transformation_matrix, alignment_transform);

    // ロボットの現在位置への変換(ICPにより計算)
    tf::Transform aligned_transform = alignment_transform * map_T_foot;

    // 初期姿勢をパブリッシュ
    publishInitialpose(aligned_transform);

    // 位置合わせ後のスキャン点群をパブリッシュ
    publishPointCloud(aligned_scan_pc);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


/**
 * @brief 占有格子地図を点群に変換
 */
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


/**
 * @brief レーザースキャンを点群に変換
 */
void CoordinateAlignment::convertScanIntoPointCloud(const sensor_msgs::LaserScanConstPtr& scan,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  sensor_msgs::PointCloud2 pc2;  // 点群に変換したスキャン
  sensor_msgs::PointCloud2 pc2_map_frame;  // 点群に変換したスキャン(map座標系)

  laser_geometry::LaserProjection projector;
  // LaserScanからPointCloud2への変換
  projector.projectLaser(*scan, pc2);
  // PointCloudの座標変換
  pcl_ros::transformPointCloud("map", pc2, pc2_map_frame, tf_listner_);
  // PCLのPointCloudへ変換
  pcl::fromROSMsg(pc2_map_frame, *scan_pc_);
}


/**
 * @brief ICPにより点群の重ね合わせ
 */
void CoordinateAlignment::alignWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_pc,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr alined_pc,
                                       Eigen::Matrix4d& transformation_matrix)
{
  // ICPの設定
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_pc);
  icp.setInputTarget(target_pc);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-9);

  // ICP実行
  icp.align(*alined_pc);

  // 位置合わせのための座標変換
  transformation_matrix = icp.getFinalTransformation().cast<double>();
}


/**
 * @brief 点群をパブリッシュ
 */
void CoordinateAlignment::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(*point_cloud, pc2);
  pc2.header.frame_id = "map";
  pointcloud_pub_.publish(pc2);
}


/**
 * @brief 初期姿勢をパブリッシュ
 */
void CoordinateAlignment::publishInitialpose(const tf::Transform transform)
{
  // pose
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  // aligned_pose.position.z = 0.182;
  // aligned_pose.orientation.x = 0.0;
  // aligned_pose.orientation.y = 0.0;
  // aligned_pose.orientation.w = 1.0;

  // initialpose
  geometry_msgs::PoseWithCovarianceStamped initialpose;
  initialpose.header.stamp = ros::Time::now();
  initialpose.header.frame_id = "map";
  initialpose.pose.pose = pose;

  // rvizからinitialposeをパブリッシュしたときと同じcovarianceを設定
  initialpose.pose.covariance[0] = 0.25;
  initialpose.pose.covariance[7] = 0.25;
  initialpose.pose.covariance[35] = 0.06853891945200942;
  initialpose_pub_.publish(initialpose);
}


/**
 * @brief Eigenの行列をTFに変換
 */
void CoordinateAlignment::convertMatrix4dIntoTF(const Eigen::Matrix4d& eigen_matrix,
                                                tf::Transform& transform)
{
  Eigen::Vector3d trans(eigen_matrix.block<3, 1>(0, 3));
  Eigen::Quaterniond q(eigen_matrix.block<3, 3>(0, 0));
  transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
  transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
}


/**
 * @brief メイン関数
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_alignment");

  CoordinateAlignment coordinate_alignment;
  ros::spin();

  return 0;
}
