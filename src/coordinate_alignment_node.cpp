#include <ros/ros.h>


class CoordinateAlignment
{
 public:
  CoordinateAlignment();

 private:
  // ノードハンドラ
  ros::NodeHandle nh_;
};

CoordinateAlignment::CoordinateAlignment()
{
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_alignment");

  CoordinateAlignment coordinate_alignment;
  ros::spin();

  return 0;
}
