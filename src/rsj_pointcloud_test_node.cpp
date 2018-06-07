#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class RsjPointcloudTestNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_points_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_transformed_;
  PointCloud::Ptr cloud_tranformed_;

  void cbPoints(const PointCloud::ConstPtr &msg)
  {
    try
    {
      std::string frame_id = msg->header.frame_id;
      PointCloud::ConstPtr cloud_src = msg;
      if (target_frame_.empty() == false)
      {
        frame_id = target_frame_;
        if (pcl_ros::transformPointCloud(target_frame_, *msg, *cloud_tranformed_, tf_listener_) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame_ = %s", target_frame_.c_str());
          return;
        }
        pub_transformed_.publish(cloud_tranformed_);
        cloud_src = cloud_tranformed_;
      }
      // ここに cloud_src に対するフィルタ処理を書く
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
  visualization_msgs::Marker makeMarker(const std::string &frame_id, const std::string &marker_ns, int marker_id, const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
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
  RsjPointcloudTestNode()
    : nh_()
    , pnh_("~")
  {
    std::string topic_name;
    pnh_.param("target_frame", target_frame_, std::string(""));
    pnh_.param("topic_name", topic_name, std::string("/camera/depth_registered/points"));
    ROS_INFO("target_frame='%s'", target_frame_.c_str());
    ROS_INFO("topic_name='%s'", topic_name.c_str());
    sub_points_ = nh_.subscribe(topic_name, 5, &RsjPointcloudTestNode::cbPoints, this);
    pub_transformed_ = nh_.advertise<PointCloud>("transform", 1);
    cloud_tranformed_.reset(new PointCloud());
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rsj_pointcloud_test_node");

  RsjPointcloudTestNode pointcloud_test;
  ROS_INFO("Hello Point Cloud!");
  ros::spin();
}
