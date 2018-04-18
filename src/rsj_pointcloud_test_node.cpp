#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
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
  pcl::PassThrough<PointT> pass;
  PointCloud::Ptr cloud_passthrough;
  ros::Publisher pub_passthrough;
  pcl::VoxelGrid<PointT> voxel;
  PointCloud::Ptr cloud_voxel;
  ros::Publisher pub_voxel;
  pcl::search::KdTree<PointT>::Ptr tree;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ros::Publisher pub_clusters;

  void cb_points(const PointCloud::ConstPtr &msg)
  {
    try
    {
      std::string frame_id = msg->header.frame_id;
      if (target_frame.empty() == false)
      {
        frame_id = target_frame;
        if (pcl_ros::transformPointCloud(target_frame, *msg, *cloud_tranform, tf_listener) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s", target_frame.c_str());
          return;
        }
      }
      pub_transform.publish(cloud_tranform);
      pass.setInputCloud(cloud_tranform);
      pass.filter(*cloud_passthrough);
      pub_passthrough.publish(cloud_passthrough);
      voxel.setInputCloud(cloud_passthrough);
      voxel.filter(*cloud_voxel);
      pub_voxel.publish(cloud_voxel);
      std::vector<pcl::PointIndices> cluster_indices;
      tree->setInputCloud(cloud_voxel);
      ec.setInputCloud(cloud_voxel);
      ec.extract(cluster_indices);
      visualization_msgs::MarkerArray marker_array;
      int marker_id = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end();
           it != it_end; ++it, ++marker_id)
      {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_voxel, *it, min_pt, max_pt);
        marker_array.markers.push_back(make_marker(frame_id, marker_id, min_pt, max_pt));
      }
      if (marker_array.markers.empty() == false)
      {
        pub_clusters.publish(marker_array);
      }
      ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu)", msg->size(), cloud_passthrough->size(), cloud_voxel->size(), cluster_indices.size());
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
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
    target_frame = "";
    nh.getParam("target_frame", target_frame);
    ROS_INFO("target_frame='%s'", target_frame.c_str());
    sub_points = nh.subscribe("/camera/depth_registered/points", 5,
                              &rsj_pointcloud_test_node::cb_points, this);
    pub_transform = nh.advertise<PointCloud>("transform", 1);
    cloud_tranform.reset(new PointCloud());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.0);
    cloud_passthrough.reset(new PointCloud());
    pub_passthrough = nh.advertise<PointCloud>("passthrough", 1);
    voxel.setLeafSize(0.025f, 0.025f, 0.025f);
    cloud_voxel.reset(new PointCloud());
    pub_voxel = nh.advertise<PointCloud>("voxel", 1);
    tree.reset(new pcl::search::KdTree<PointT>());
    ec.setClusterTolerance(0.15);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    pub_clusters = nh.advertise<visualization_msgs::MarkerArray>("clusters", 1);
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
