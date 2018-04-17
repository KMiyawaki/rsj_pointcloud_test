#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class rsj_pointcloud_test_node
{
private:
  ros::Subscriber sub_points;
  pcl::PassThrough<PointT> pass;
  pcl::PointCloud<PointT>::Ptr cloud_passthrough;
  ros::Publisher pub_passthrough;

  void cb_points(const PointCloud::ConstPtr &msg)
  {
    pass.setInputCloud(msg);
    pass.filter(*cloud_passthrough);
    pub_passthrough.publish(cloud_passthrough);
    ROS_INFO("points (src: %zu, paththrough: %zu)", msg->size(), cloud_passthrough->size());
  }

public:
  rsj_pointcloud_test_node()
  {
    ros::NodeHandle nh("~");
    sub_points = nh.subscribe("/camera/depth_registered/points", 5,
                              &rsj_pointcloud_test_node::cb_points, this);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, -0.5);
    cloud_passthrough.reset(new PointCloud);
    pub_passthrough = nh.advertise<PointCloud>("passthrough", 1);
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
