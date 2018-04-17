#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class rsj_pointcloud_test_node
{
private:
  ros::Subscriber sub_points;

  void cb_points(const PointCloud::ConstPtr &msg)
  {
  }
  
  visualization_msgs::Marker make_marker(const std::string &frame_id, int marker_id, const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pcl_clusters";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2f;

    marker.lifetime = ros::Duration(0.3);
    return marker;
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
