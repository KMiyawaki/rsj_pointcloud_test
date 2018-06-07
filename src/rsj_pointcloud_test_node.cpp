#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  // 追記
#include <pcl/filters/voxel_grid.h>  // 追記
#include <pcl/common/common.h>  // 追記
#include <pcl/kdtree/kdtree.h>  // 追記
#include <pcl/segmentation/extract_clusters.h>  // 追記
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class rsj_pointcloud_test_node
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_points_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_transformed_;
  PointCloud::Ptr cloud_tranformed_;
  // 以下を追記
  pcl::PassThrough<PointT> pass_;
  PointCloud::Ptr cloud_passthrough_;
  ros::Publisher pub_passthrough_;
  // 以下を追記
  pcl::VoxelGrid<PointT> voxel_;
  PointCloud::Ptr cloud_voxel_;
  ros::Publisher pub_voxel_;
  // 以下を追記
  pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::EuclideanClusterExtraction<PointT> ec_;
  ros::Publisher pub_clusters_;

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
      pass_.setInputCloud(cloud_src);
      pass_.filter(*cloud_passthrough_);
      pub_passthrough_.publish(cloud_passthrough_);
      // 以下のように追記・修正
      voxel_.setInputCloud(cloud_passthrough_);
      voxel_.filter(*cloud_voxel_);
      pub_voxel_.publish(cloud_voxel_);
      // 以下のように追記・修正
      std::vector<pcl::PointIndices> cluster_indices;
      tree_->setInputCloud(cloud_voxel_);
      ec_.setInputCloud(cloud_voxel_);
      ec_.extract(cluster_indices);
      visualization_msgs::MarkerArray marker_array;
      int target_index = -1;  // 追記
      int marker_id = 0;
      size_t ok = 0;  // 追記
      for (std::vector<pcl::PointIndices>::const_iterator 
               it = cluster_indices.begin(),
               it_end = cluster_indices.end();            
           it != it_end; ++it, ++marker_id)
      {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_voxel_, *it, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
        {
         // 以下を追記・修正
          bool is_ok = true;
          if (cluster_size.x() < 0.05 || cluster_size.x() > 0.4)
          {
            is_ok = false;
          }
          else if (cluster_size.y() < 0.05 || cluster_size.y() > 0.6)
          {
            is_ok = false;
          }
          else if (cluster_size.z() < 0.05 || cluster_size.z() > 0.5)
          {
            is_ok = false;
          }
          visualization_msgs::Marker marker =
              makeMarker(
                  frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);
          if (is_ok)
          {
            marker.ns = "ok_cluster";
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            ok++;
                      // 以下のように追記
            if(target_index < 0){
              target_index = marker_array.markers.size();
            }else{
              float d1 = ::hypot(marker_array.markers[target_index].pose.position.x,
                                 marker_array.markers[target_index].pose.position.y);
              float d2 = ::hypot(marker.pose.position.x, marker.pose.position.y);
              if(d2 < d1){
                target_index = marker_array.markers.size();
              }
            }
          }
          marker_array.markers.push_back(marker);
          // 追記・修正箇所ここまで
        }
      }
      if (marker_array.markers.empty() == false)
      {
        // 以下のように追記
        if(target_index >= 0){
          marker_array.markers[target_index].ns = "target_cluster";
          marker_array.markers[target_index].color.r = 1.0f;
          marker_array.markers[target_index].color.g = 0.0f;
          marker_array.markers[target_index].color.b = 1.0f;
          marker_array.markers[target_index].color.a = 0.5f;
        }
        // 追記箇所ここまで
        pub_clusters_.publish(marker_array);
      }
      ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu)",
               msg->size(), cloud_passthrough_->size(), cloud_voxel_->size(),
               cluster_indices.size());
      // 追記・修正箇所ここまで
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
    std::string topic_name;
    pnh_.param("target_frame", target_frame_, std::string(""));
    pnh_.param("topic_name", topic_name, std::string("/camera/depth_registered/points"));
    ROS_INFO("target_frame = '%s'", target_frame_.c_str());
    ROS_INFO("topic_name = '%s'", topic_name.c_str());
    sub_points_ = nh_.subscribe(topic_name, 5, &RsjPointcloudTestNode::cbPoints, this);
    pub_transformed_ = nh_.advertise<PointCloud>("cloud_transformed", 1);
    cloud_tranformed_.reset(new PointCloud());
    // 以下を追記
    pass_.setFilterFieldName("z");  // Z軸（高さ）の値でフィルタをかける
    pass_.setFilterLimits(0.1, 1.0);  // 0.1 ～ 1.0 m の間にある点群を抽出
    cloud_passthrough_.reset(new PointCloud());
    pub_passthrough_ = nh_.advertise<PointCloud>("passthrough", 1);
     // 以下を追記
    voxel_.setLeafSize(0.025f, 0.025f, 0.025f);  // 0.025 m 間隔でダウンサンプリング
    cloud_voxel_.reset(new PointCloud());
    pub_voxel_ = nh_.advertise<PointCloud>("voxel", 1);
    // 以下を追記
    tree_.reset(new pcl::search::KdTree<PointT>());
    ec_.setClusterTolerance(0.15);
    ec_.setMinClusterSize(100);
    ec_.setMaxClusterSize(5000);
    ec_.setSearchMethod(tree_);
    pub_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rsj_pointcloud_test_node");
  rsj_pointcloud_test_node pointcloud_test;
  pointcloud_test.mainloop();
}
