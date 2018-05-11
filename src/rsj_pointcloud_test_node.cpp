#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class rsj_pointcloud_test_node
{
private:
  ros::Subscriber sub_points;
  std::string target_frame;
  tf::TransformListener tf_listener;
  ros::Publisher pub_transform;
  PointCloud::Ptr cloud_tranform;

  void cb_points(const PointCloud::ConstPtr &msg)
  {
    try
    {
      std::string frame_id = msg->header.frame_id;
      PointCloud::ConstPtr cloud_src = msg;
      if (target_frame.empty() == false)
      {
        frame_id = target_frame;
        if (pcl_ros::transformPointCloud(target_frame, *msg, *cloud_tranform, tf_listener) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s", target_frame.c_str());
          return;
        }
        pub_transform.publish(cloud_tranform);
        cloud_src = cloud_tranform;
      }
      // ここに cloud_src に対するフィルタ処理を書く
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
  visualization_msgs::Marker make_marker(const std::string &frame_id, const std::string &marker_ns, int marker_id, const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
                                         float r, float g, float b, float a) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
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

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.3);
    return marker;
  }

public:
  rsj_pointcloud_test_node()
  {
    ros::NodeHandle nh("~");
    target_frame = "";
    std::string topic_name = "/camera/depth_registered/points";
    nh.getParam("target_frame", target_frame);
    nh.getParam("topic_name", topic_name);
    ROS_INFO("target_frame='%s'", target_frame.c_str());
    ROS_INFO("topic_name='%s'", topic_name.c_str());
    sub_points = nh.subscribe(topic_name, 5, &rsj_pointcloud_test_node::cb_points, this);
    pub_transform = nh.advertise<PointCloud>("transform", 1);
    cloud_tranform.reset(new PointCloud());
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
