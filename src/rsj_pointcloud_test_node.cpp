#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class rsj_pointcloud_test_node
{
private:
  ros::Subscriber sub_points;

  void cb_points(const PointCloud::ConstPtr &msg)
  {
    ROS_INFO("width: %d, height: %d", msg->width, msg->height);
  }

public:
  rsj_pointcloud_test_node()
  {
    ros::NodeHandle nh("~");
    sub_points = nh.subscribe("/camera/depth_registered/points", 5,
                              &rsj_pointcloud_test_node::cb_points, this);
  }

  void mainloop()
  {
    ROS_INFO("Hello Point Cloud!");

    ros::Rate rate(30.0);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rsj_pointcloud_test_node");
  rsj_pointcloud_test_node pointcloud_test;
  pointcloud_test.mainloop();
}
