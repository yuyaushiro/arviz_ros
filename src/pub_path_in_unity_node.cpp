#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class PubPathInUnity
{
 public:
  PubPathInUnity();
  void pathCallback(const nav_msgs::PathConstPtr& path);

 private:
  ros::NodeHandle nh_;

  tf::TransformListener tf_listener_;

  ros::Subscriber path_sub_;
  ros::Publisher path_pub_;
};

PubPathInUnity::PubPathInUnity()
{
  path_sub_ = nh_.subscribe<nav_msgs::Path>("move_base/NavfnROS/plan", 1, &PubPathInUnity::pathCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("unity/plan", 1, false);
}

void PubPathInUnity::pathCallback(const nav_msgs::PathConstPtr& path)
{
  try
  {
    tf_listener_.waitForTransform("unity", path->header.frame_id,
                                  ros::Time::now(), ros::Duration(1.0));
    // メッセージの準備
    nav_msgs::Path path_out;
    path_out.header.stamp = ros::Time::now();
    path_out.header.frame_id = "unity";
    geometry_msgs::PoseStamped pose_stamped;

    for (auto itr = path->poses.begin(); itr != path->poses.end(); ++itr)
    {
      if (itr->pose.position.z == 0)
      {
        tf_listener_.transformPose("unity", *itr, pose_stamped);
        path_out.poses.push_back(pose_stamped);
      }
    }
    // publish
    path_pub_.publish(path_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_path_in_unity");
  PubPathInUnity pub_path_in_unity;
  ros::spin();

  return 0;
}
