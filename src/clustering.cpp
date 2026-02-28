#include <memory>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

#include <dwal_planner/libdwal_cluster.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <dwal_planner/msg/cluster_group.hpp>
#include <dwal_planner/msg/sampled_cluster.hpp>
#include <dwal_planner/msg/path_cluster.hpp>
#include <dwal_planner/msg/path.hpp>
#include <dwal_planner/msg/pose2_d32.hpp>

#include <dwal_planner/srv/toggle_clusters.hpp>
#include <dwal_planner/srv/toggle_slice.hpp>

using std::placeholders::_1;

double colors[] = {
  1, 0, 0,  0, 1, 0,  0, 0, 1,  1, 1, 0,
  0, 1, 1,  1, 0, 1,  0.5, 0.5, 0.2,  0.8, 0.1, 0.4
};

void setColor(visualization_msgs::msg::Marker & path_marker, int c, double cost)
{
  path_marker.color.r = colors[3 * c] * cost;
  path_marker.color.g = colors[3 * c + 1] * cost;
  path_marker.color.b = colors[3 * c + 2] * cost;
}

class Clustering
{
public:
  explicit Clustering(const std::shared_ptr<rclcpp::Node>& node)
  : node_(node)
  {
    prev_cluster_num = 0;
    source_slice_group_ptr = nullptr;
    source_slice_cluster_index = 1;
    sliced_path_indexes.clear();
    do_slice = false;
    direction = false;

    node_->get_parameter("dwal_clustering/min_cluster_span", min_cluster_span);
    node_->get_parameter("dwal_clustering/cluster_separation", cluster_separation);

    std::string odom_frame;
    node_->declare_parameter<std::string>("common/odom_frame", "odom");
    node_->get_parameter("common/odom_frame", odom_frame);
    RCLCPP_INFO(node_->get_logger(), "Using odometry frame '%s'", odom_frame.c_str());
    
    // Wait until the generator sets a non-negative value
    marker_num = -1;
    while (rclcpp::ok() && marker_num < 0) {
      (void)node_->get_parameter("dwal_clustering/Marker_num", marker_num);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "Waiting for parameter 'dwal_clustering/Marker_num' to be set...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(), "Marker_num received: %d", marker_num);
    // Init marker template
    path_marker.header.frame_id = odom_frame;
    path_marker.header.stamp = node_->now();
    path_marker.ns = "dwal_planner";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.01;
    path_marker.color.r = 0.7;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.8;

    path_array_marker.markers.reserve(static_cast<size_t>(marker_num));
    for (int k = 0; k < marker_num; k++) {
      path_marker.id = k;
      path_array_marker.markers.push_back(path_marker);
    }
  }

  void set_publishers(const std::string& marker_topic, const std::string& cluster_topic)
  {
    markers_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(2));
    group_publisher   = node_->create_publisher<dwal_planner::msg::ClusterGroup>(cluster_topic, rclcpp::QoS(2));
  }

  int clusterIdtoIndex(const int& id, const dwal_planner::msg::ClusterGroup* cl_group)
  {
    for (int k = 0; k < static_cast<int>(cl_group->clusters.size()); k++)
      if (cl_group->clusters[k].id == id)
        return k;
    return -1;
  }

  void findCorrespondence()
  {
    corresp_matrix.clear();
    if (cluster_num == 0) //current frame empty
    {
      prev_ids.clear();
      prev_mean_phi.clear();
      prev_cluster_num = cluster_num;
      return;
    }

    if (prev_cluster_num == 0) //previous frame empty
    {
      prev_ids.clear();
      prev_mean_phi.clear();
      prev_cluster_num = cluster_num;
      unsigned int i = 0;
      for (auto &cl : group.clusters)
      {
        cl.id = static_cast<int>(i);
        prev_mean_phi.push_back((cl.paths.front().phi + cl.paths.back().phi) / 2);
        prev_ids.push_back(i);
        i++;
      }
      return;
    }

    cur_mean_phi.clear();
    phi_distances.clear();
    temp.clear();

    //calculate mean phis for current clusters
    for (auto &cl : group.clusters)
      cur_mean_phi.push_back((cl.paths.front().phi + cl.paths.back().phi) / 2);

    temp.resize(static_cast<size_t>(cluster_num)); //calculate mean phi distance between clusters
    for (int k = 0; k < prev_cluster_num; k++)
    {
      phi_distances.push_back(temp);
      for (int i = 0; i < cluster_num; i++)
        phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)] = std::fabs(cur_mean_phi[static_cast<size_t>(i)] - prev_mean_phi[static_cast<size_t>(k)]);
    }

    int row, col; //find minimum phi distance
    double minimum;
    for (int j = 0; j < std::min(prev_cluster_num, cluster_num); j++)
    {
      row = col = 0;
      minimum = 1000;
      for (int k = 0; k < prev_cluster_num; k++)
      {
        for (int i = 0; i < cluster_num; i++)
        {
          if (phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)] < minimum)
          {
            minimum = phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)];
            row = k;
            col = i;
          }
        }
      }
      group.clusters[static_cast<size_t>(col)].id = static_cast<int>(prev_ids[static_cast<size_t>(row)]); //assign same id to min distance
      corresp_matrix[prev_ids[static_cast<size_t>(row)]] = static_cast<unsigned int>(col); // prev ID -> current INDEX

      //invalidate selected row and col by assigning distance=1000
      for (int k = 0; k < prev_cluster_num; k++)
        phi_distances[static_cast<size_t>(k)][static_cast<size_t>(col)] = 1000;
      std::fill(phi_distances[static_cast<size_t>(row)].begin(), phi_distances[static_cast<size_t>(row)].end(), 1000);
    }

    if (cluster_num > prev_cluster_num)
    {
      unsigned int j = 0; //for the rest of the clusters assign appropriate id
      std::vector<unsigned int>::iterator it;
      it = std::find(prev_ids.begin(), prev_ids.end(), j);
      for (auto &cl : group.clusters)
      {
        j = 0;
        if (cl.id == 100)
        {
          while (it != prev_ids.end())
          {
            j++;
            it = std::find(prev_ids.begin(), prev_ids.end(), j);
          }
          cl.id = static_cast<int>(j);
          prev_ids.push_back(j);
        }
      }
    }
    prev_ids.clear();
    prev_mean_phi.clear();
    prev_cluster_num = cluster_num;
    prev_mean_phi = cur_mean_phi;
    for (int i = 0; i < cluster_num; i++)
      prev_ids.push_back(static_cast<unsigned int>(group.clusters[static_cast<size_t>(i)].id));
  }

  void doClustering(const dwal_planner::msg::SampledCluster::SharedPtr msg)
  {
    //msg contains all sampled paths, for the given level, up to their collision point (if one exists)
    cluster_num = 0;
    Rcl = levels[static_cast<size_t>(level_index)];
    path_num = static_cast<int>(msg->paths.size());
    free_path_indexes.clear();
    cluster_indexes.clear();
    cluster_vecs.clear();
    group.clusters.clear();
    for (int k = 0; k < marker_num; k++)
      path_array_marker.markers[static_cast<size_t>(k)].points.clear(); //clear all path markers

    for (int k = 0; k < path_num; k++)
    { //get free paths
      if (msg->paths[static_cast<size_t>(k)].colission_r > Rcl)
        free_path_indexes.push_back(k);
    }

    if (free_path_indexes.empty()) //do we have at least 1 valid path?
    {
      group_publisher->publish(group); //publish empty clusters
      markers_publisher->publish(path_array_marker); //publish empty markers and exit
      return;
    }

    // do cluster slicing with source group, if flag is raised
    //////--- start slicing -----------------------------------------------------------
    if (do_slice && source_slice_group_ptr && !source_slice_group_ptr->clusters.empty())
    {
      source_slice_cluster_index = clusterIdtoIndex(source_slice_cluster_id, source_slice_group_ptr);
      if (source_slice_cluster_index == -1)
      {
        RCLCPP_INFO(node_->get_logger(), "--- Source cluster ID not found. Skipping slicing...");
        return;
      }
      switch (direction)
      {
        case false: //false = right
          max_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index)].paths.back().curvature;
          if (source_slice_cluster_index > 0)
            min_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index - 1)].paths.back().curvature;
          else
            min_curv_slice = msg->paths.front().curvature;
          break;

        case true: //true = left
          min_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index)].paths[0].curvature;
          if (source_slice_cluster_index < static_cast<int>(source_slice_group_ptr->clusters.size()) - 1)
            max_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index + 1)].paths[0].curvature;
          else
            max_curv_slice = msg->paths.back().curvature;
          break;
      }

      for (int &k : free_path_indexes)
        if (msg->paths[static_cast<size_t>(k)].curvature >= min_curv_slice &&
            msg->paths[static_cast<size_t>(k)].curvature <= max_curv_slice)
          sliced_path_indexes.push_back(k);

      free_path_indexes = sliced_path_indexes;
      sliced_path_indexes.clear();
    }
    //////--- end slicing ----------------------------------------------------------------

    //////--- start clustering -----------------------------------------------------------
    cluster_indexes.push_back(free_path_indexes[0]); //1st path is always a cluster border
    phi0 = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(free_path_indexes[0])].curvature, Rcl);
    for (int k = 1; k < static_cast<int>(free_path_indexes.size()); k++)
    {
      Dind = free_path_indexes[static_cast<size_t>(k)] - cluster_indexes.back();
      if (Dind > cluster_separation) //gap found
      {
        Dphip = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(cluster_indexes.back())].curvature, Rcl) - phi0;
        if (cluster_lib::clusterChord(Rcl, Dphip) >= min_cluster_span) //is last cluster big enough?
          cluster_vecs.push_back(cluster_indexes); //push it to cluster vecs
        cluster_indexes.clear(); //clear to begin new cluster
        phi0 = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(free_path_indexes[static_cast<size_t>(k)])].curvature, Rcl);
        cluster_indexes.push_back(free_path_indexes[static_cast<size_t>(k)]); //add current path
      }
      else if (Dind <= 2)
        cluster_indexes.push_back(free_path_indexes[static_cast<size_t>(k)]); //add current path
    }

    Dphip = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(free_path_indexes.back())].curvature, Rcl) - phi0;
    if (cluster_lib::clusterChord(Rcl, Dphip) >= min_cluster_span) //check last curv
      cluster_vecs.push_back(cluster_indexes);

    cluster_num = static_cast<int>(cluster_vecs.size());
    //////--- end clustering -----------------------------------------------------------

    //populate cluster_group msg and path markers
    int j;
    for (std::vector<int> &clvec : cluster_vecs)
    {
      tmp_cluster.paths.clear(); //clear temp cluster
      tmp_cluster.id = 100; //start with id=100. Will change in correspondence matching later
      tmp_cluster.r = Rcl;
      tmp_cluster.pose0 = msg->pose0;
      
      for (int &l : clvec)
      {
        j = 0;
        if (*spin != 0)
        {
          for (const dwal_planner::msg::Pose2D32 &p : msg->paths[static_cast<size_t>(l)].poses)
          {
            p0.x = p.x;
            p0.y = p.y;
            path_array_marker.markers[static_cast<size_t>(l)].points.push_back(p0);
            
            if ( j >= static_cast<int>(msg->paths[l].level_inds[level_index]) )
              break;
            j++;
          }
        }
        tmp_path.curvature = msg->paths[static_cast<size_t>(l)].curvature;
        tmp_path.phi = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(l)].curvature, Rcl);
        tmp_path.cost = msg->paths[static_cast<size_t>(l)].costs[static_cast<size_t>(level_index)];
        tmp_cluster.paths.push_back(tmp_path); //add path to current cluster    
      }
      group.clusters.push_back(tmp_cluster);
    }

    findCorrespondence(); //set cluster ids according to correspondence matching

    if (*spin != 0)
    {
      //draw clusters with different color
      for (int k = 0; k < cluster_num; k++)
        for (int l : cluster_vecs[static_cast<size_t>(k)])
          setColor(path_array_marker.markers[static_cast<size_t>(l)],
                   group.clusters[static_cast<size_t>(k)].id,
                   0.3 + 0.7 * (static_cast<double>(msg->paths[static_cast<size_t>(l)].costs[static_cast<size_t>(level_index)]) / 128.0));
    }

    //publish markers & groups
    group_publisher->publish(group);
    markers_publisher->publish(path_array_marker);
  }

  std::vector<int>::iterator spin;
  int level_index{0};
  std::vector<double> levels;

  std::map<unsigned int, unsigned int> corresp_matrix;
  dwal_planner::msg::ClusterGroup group;
  dwal_planner::msg::ClusterGroup* source_slice_group_ptr{nullptr};
  int source_slice_cluster_id{0};
  int source_slice_cluster_index{1};
  bool do_slice{false};
  bool direction{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
  rclcpp::Publisher<dwal_planner::msg::ClusterGroup>::SharedPtr group_publisher;

private:
  std::shared_ptr<rclcpp::Node> node_;

  geometry_msgs::msg::Point p0;
  visualization_msgs::msg::Marker path_marker;
  visualization_msgs::msg::MarkerArray path_array_marker;

  double Dphip{0.0}, min_cluster_span{0.5}, Rcl{0.0}, curv_mob{0.0}, phi0{0.0};
  std::vector<double> temp, prev_mean_phi, cur_mean_phi;

  std::vector<int> free_path_indexes, sliced_path_indexes, cluster_indexes;
  std::vector<std::vector<int>> cluster_vecs;
  std::vector<unsigned int> prev_ids;
  std::vector<std::vector<double>> phi_distances;
  int marker_num{0}, cluster_separation{5}, cluster_num{0}, path_num{0}, prev_cluster_num{0}, Dind{0};

  dwal_planner::msg::PathCluster tmp_cluster;
  dwal_planner::msg::Path tmp_path;
  double min_curv_slice{0.0}, max_curv_slice{0.0};
};

/* main class that handles the sampled paths. It creates Clustering objects for each level (group)
   and performs the clustering by calling their appropriate method */
class ControlClass
{
public:
  std::shared_ptr<rclcpp::Node> ros_node;
  explicit ControlClass(const std::shared_ptr<rclcpp::Node>& node)
  : ros_node(node)
  {
    markerTopic = "cluster_markers_";
    clusterTopic = "clusters_";

    ros_node->declare_parameter<std::vector<std::string>>("dwal_clustering/postfix", {});
    ros_node->declare_parameter<std::vector<double>>("common/levels", {});
    ros_node->declare_parameter<std::vector<int64_t>>("dwal_clustering/spin", {});
    ros_node->declare_parameter<double>("dwal_clustering/min_cluster_span", 0.5);
    ros_node->declare_parameter<int>("dwal_clustering/cluster_separation", 5);
    ros_node->declare_parameter<int>("dwal_clustering/Marker_num", -1);

    ros_node->get_parameter("dwal_clustering/postfix", postfixes);
    ros_node->get_parameter("common/levels", levels);
    ros_node->get_parameter("dwal_clustering/spin", spin64);


    spin.clear();
    spin.reserve(spin64.size());
    for (auto v : spin64) spin.push_back(static_cast<int>(v));

    group_num = postfixes.size();
    if (group_num != levels.size())
    {
      RCLCPP_ERROR(ros_node->get_logger(),
                   "\n\n----ERROR : 'postfix' and 'levels' should have the same number of elements...\n"
                   "----aborting launch of clustering node...\n");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(ros_node->get_logger(), "Cluster groups to be created: %zu", group_num);
    RCLCPP_INFO(ros_node->get_logger(), "Cluster postfixes: %s (%.2f m), %s (%.2f m)", postfixes[0].c_str(),levels[0], postfixes[1].c_str(),levels[1]);

    do_slice = false;
    group_objects.reserve(group_num);
    for (size_t k = 0; k < group_num; k++) {
      group_objects.emplace_back(ros_node);  
      clustering_order.push_back(static_cast<int>(k));
      group_objects[k].level_index = static_cast<int>(k);
      group_objects[k].levels = levels;
      group_objects[k].spin = spin.begin() + static_cast<long>(k);
      const std::string marker_topic  = markerTopic  + postfixes[k];
      const std::string clusters_topic = clusterTopic + postfixes[k];
      group_objects[k].set_publishers(marker_topic, clusters_topic);
    }

    // Subscriber
    sampled_cluster_subscriber = ros_node->create_subscription<dwal_planner::msg::SampledCluster>(
      "sampled_paths", rclcpp::QoS(1),
      std::bind(&ControlClass::sampledClusterHandler, this, _1));

    // Services
    toggleSpinService = ros_node->create_service<dwal_planner::srv::ToggleClusters>(
      "toggle_cluster_spin",
      [this](const std::shared_ptr<dwal_planner::srv::ToggleClusters::Request> req,
             std::shared_ptr<dwal_planner::srv::ToggleClusters::Response> res)
      {
        for (size_t k = 0; k < req->flags.size() && k < spin.size(); k++)
          spin[k] = req->flags[k];
        res->success = true;
      });

    toggleSliceService = ros_node->create_service<dwal_planner::srv::ToggleSlice>(
      "toggle_cluster_slice",
      [this](const std::shared_ptr<dwal_planner::srv::ToggleSlice::Request> req,
             std::shared_ptr<dwal_planner::srv::ToggleSlice::Response> res)
      {
        clustering_order.clear();
        clustering_order.push_back(req->source_group_index);
        clustering_order.push_back(req->target_group_index);
        for (int k = 0; k < static_cast<int>(group_num); k++)
        {
          group_objects[static_cast<size_t>(k)].do_slice = false; // reset slicing flags
          do_slice = false;
          if ((k != req->source_group_index) && (k != req->target_group_index))
            clustering_order.push_back(k);
        }

        if (req->do_slice)
        {
          group_objects[static_cast<size_t>(req->target_group_index)].do_slice = true;
          group_objects[static_cast<size_t>(req->target_group_index)].direction = req->direction;
          group_objects[static_cast<size_t>(req->target_group_index)].source_slice_cluster_id = req->source_cluster_id;
          do_slice = true;
        }
        res->success = true;
      });
  }

  void sampledClusterHandler(const dwal_planner::msg::SampledCluster::SharedPtr msg)
  {
    if (do_slice)
    {
      Clustering & src_cluster = group_objects[static_cast<size_t>(clustering_order[0])];
      Clustering & trg_cluster = group_objects[static_cast<size_t>(clustering_order[1])];

      src_cluster.doClustering(msg); //do src group clustering
      trg_cluster.source_slice_group_ptr = &src_cluster.group;

      // check to see if prev src frame was empty
      if (!src_cluster.corresp_matrix.empty())
      {
        trg_cluster.source_slice_cluster_index =
          static_cast<int>(src_cluster.corresp_matrix[static_cast<unsigned int>(trg_cluster.source_slice_cluster_id)]);
        trg_cluster.source_slice_cluster_id =
          src_cluster.group.clusters[static_cast<size_t>(trg_cluster.source_slice_cluster_index)].id;
      }
      else
        trg_cluster.source_slice_cluster_index = -1;

      trg_cluster.doClustering(msg); //do target group clustering

      for (size_t k = 2; k < group_num; k++)
        group_objects[static_cast<size_t>(clustering_order[static_cast<size_t>(k)])].doClustering(msg);
    }
    else
    {
      for (size_t k = 0; k < group_num; k++)
        group_objects[static_cast<size_t>(clustering_order[k])].doClustering(msg);
    }
  }


  // ROS2 entities
  rclcpp::Subscription<dwal_planner::msg::SampledCluster>::SharedPtr sampled_cluster_subscriber;
  rclcpp::Service<dwal_planner::srv::ToggleClusters>::SharedPtr toggleSpinService;
  rclcpp::Service<dwal_planner::srv::ToggleSlice>::SharedPtr toggleSliceService;

  // State kept the same
  std::vector<int> spin, clustering_order;
  std::vector<int64_t> spin64;
  std::vector<Clustering> group_objects;
  bool do_slice{false};
  size_t group_num{0};
  std::vector<std::string> postfixes;
  std::vector<double> levels;
  std::string markerTopic, clusterTopic;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dwal_clustering");

  ControlClass ctrlObj(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
