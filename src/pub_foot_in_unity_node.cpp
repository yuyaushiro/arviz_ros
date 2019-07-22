#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>


class PubFootInUnity
{
 public:
  PubFootInUnity();
  void timerCallback(const ros::TimerEvent&);

 private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  ros::Publisher foot_pub_;
  ros::Timer timer_;
};

PubFootInUnity::PubFootInUnity()
{
  foot_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("unity/foot", 1, true);
  timer_ = nh_.createTimer(ros::Duration(0.1), &PubFootInUnity::timerCallback, this);
}

void PubFootInUnity::timerCallback(const ros::TimerEvent&)
{
  try
  {
    tf_listener_.waitForTransform("unity", "base_footprint",
                                  ros::Time::now(), ros::Duration(3.0));
    tf::StampedTransform unity_T_foot;
    tf_listener_.lookupTransform("unity", "base_footprint", ros::Time(0), unity_T_foot);
    // メッセージの準備
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "unity";
    geometry_msgs::Pose pose;
    
    tf::poseTFToMsg(unity_T_foot, pose);
    pose_stamped.pose = pose;
    // publish
    foot_pub_.publish(pose_stamped);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_foot_in_unity");
  PubFootInUnity pub_foot_in_unity;
  ros::spin();

  return 0;
}
