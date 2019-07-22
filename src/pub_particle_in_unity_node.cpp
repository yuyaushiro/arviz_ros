#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>


class PubParticleInUnity
{
 public:
  PubParticleInUnity();
  void particleCallback(const geometry_msgs::PoseArrayConstPtr& pc);
  void robotPoseCallback(const geometry_msgs::PoseStampedConstPtr& rp);

 private:
  ros::NodeHandle nh_;

  tf::TransformListener tf_listener_;

  ros::Subscriber particle_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Publisher particle_pub_;

  geometry_msgs::PoseArray pc_;
};

PubParticleInUnity::PubParticleInUnity()
{
  particle_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("particlecloud", 1, &PubParticleInUnity::particleCallback, this);
  robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("unity/robot_pose", 1, &PubParticleInUnity::robotPoseCallback, this);
  particle_pub_ = nh_.advertise<geometry_msgs::PoseArray>("unity/particlecloud", 1, true);
}

void PubParticleInUnity::robotPoseCallback(const geometry_msgs::PoseStampedConstPtr& rp)
{
  // unityからロボットの座標が送信されたとき（unity座標とmap座標のキャリブレーション）
  // 再度パーティクルを送信する
  try
  {
    tf_listener_.waitForTransform("unity", pc_.header.frame_id,
                                  ros::Time::now(), ros::Duration(1.0));
    // unity座標系からmap座標系への変換
    tf::StampedTransform unity_T_map;
    tf_listener_.lookupTransform("unity", pc_.header.frame_id,
                                  ros::Time(0), unity_T_map);
    // メッセージの準備
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "unity";
    geometry_msgs::Pose pose;

    // map座標系でのparticleの位置;
    tf::Transform map_T_particle;
    // unity座標系でのparticleの位置
    tf::Transform unity_T_particle;
    for (auto itr = pc_.poses.begin(); itr != pc_.poses.end(); ++itr)
    {
      tf::poseMsgToTF(*itr, map_T_particle);
      unity_T_particle = unity_T_map * map_T_particle;
      tf::poseTFToMsg(unity_T_particle, pose);
      pose_array.poses.push_back(pose);
      if (pose_array.poses.size() > 1000) break;
    }
    // publish
    particle_pub_.publish(pose_array);
    ros::Duration(2.0).sleep();
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void PubParticleInUnity::particleCallback(const geometry_msgs::PoseArrayConstPtr& pc)
{
  try
  {
    tf_listener_.waitForTransform("unity", pc->header.frame_id,
                                  ros::Time(0), ros::Duration(1.0));
    // unity座標系からmap座標系への変換
    tf::StampedTransform unity_T_map;
    tf_listener_.lookupTransform("unity", pc->header.frame_id,
                                  ros::Time(0), unity_T_map);
    // パーティクルを保存しておく
    pc_ = *pc;

    // メッセージの準備
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "unity";
    geometry_msgs::Pose pose;

    // map座標系でのparticleの位置;
    tf::Transform map_T_particle;
    // unity座標系でのparticleの位置
    tf::Transform unity_T_particle;
    for (auto itr = pc->poses.begin(); itr != pc->poses.end(); ++itr)
    {
      tf::poseMsgToTF(*itr, map_T_particle);
      unity_T_particle = unity_T_map * map_T_particle;
      tf::poseTFToMsg(unity_T_particle, pose);
      pose_array.poses.push_back(pose);
      if (pose_array.poses.size() > 1000) break;
    }
    // publish
    particle_pub_.publish(pose_array);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_particle_in_unity");

  PubParticleInUnity pub_particle_in_unity;
  ros::spin();

  return 0;
}
