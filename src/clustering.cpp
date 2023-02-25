#include <dwal_planner/libdwal_cluster.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <dwal_planner/Cluster_Group.h>
#include <dwal_planner/Sampled_Cluster.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <std_msgs/Int32MultiArray.h>
#include "dwal_planner/toggleClusters.h"
#include "dwal_planner/toggleSlice.h"

double colors[] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0.5, 0.5, 0.2, 0.8, 0.1, 0.4};

void setColor(visualization_msgs::Marker &path_marker, int c, double cost)
{
  path_marker.color.r = colors[3 * c] * cost;
  path_marker.color.g = colors[3 * c + 1] * cost;
  path_marker.color.b = colors[3 * c + 2] * cost;
}

class Clustering
{
public:
  ros::NodeHandle nh;
  ros::Publisher markers_publisher, group_publisher;
  geometry_msgs::Point p0;
  visualization_msgs::Marker path_marker;
  visualization_msgs::MarkerArray path_array_marker;
  std::vector<int>::iterator spin;

  double Dphip, min_cluster_span, Rcl, curv_mob, phi0;
  std::vector<double> temp, prev_mean_phi, cur_mean_phi, levels;

  std::vector<int> free_path_indexes, sliced_path_indexes, cluster_indexes;
  std::vector<std::vector<int> > cluster_vecs;
  std::vector<uint> prev_ids;
  std::map<uint, uint> corresp_matrix;
  std::vector<std::vector<double> > phi_distances;
  int marker_num, cluster_separation, cluster_num, path_num, prev_cluster_num, level_index, Dind;
  dwal_planner::Cluster_Group group;
  dwal_planner::Path_Cluster tmp_cluster;
  dwal_planner::Path tmp_path;
  bool do_slice, direction;
  dwal_planner::Cluster_Group* source_slice_group_ptr;
  int source_slice_cluster_id;
  int source_slice_cluster_index;
  double min_curv_slice, max_curv_slice;
  Clustering() // @suppress("Class members should be properly initialized")
  {

    prev_cluster_num = 0;
    source_slice_group_ptr = NULL;
    source_slice_cluster_index = 1;
    sliced_path_indexes.clear();
    do_slice = direction = false;
    nh.param("dwal_clustering/min_cluster_span", min_cluster_span, 0.5);
    nh.param("dwal_clustering/cluster_separation", cluster_separation, 5);

    while (!nh.getParam("dwal_clustering/Marker_num", marker_num))
    {
    }; //wait until marker num becomes available

    path_marker.header.frame_id = "odom";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "dwal_planner";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.01;
    path_marker.color.r = 0.7;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.8;
    for (int k = 0; k < marker_num; k++)
    {
      path_marker.id = k;
      path_array_marker.markers.push_back(path_marker);
    }
  }

  int clusterIdtoIndex(const int& id, const dwal_planner::Cluster_Group* cl_group)
  {
    for (int k = 0; k < (int)cl_group->clusters.size(); k++)
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
        cl.id = i;
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

    temp.resize(cluster_num); //calculate mean phi distance between clusters
    for (int k = 0; k < prev_cluster_num; k++)
    {
      phi_distances.push_back(temp);
      for (int i = 0; i < cluster_num; i++)
        phi_distances[k][i] = fabs(cur_mean_phi[i] - prev_mean_phi[k]);
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
          if (phi_distances[k][i] < minimum)
          {
            minimum = phi_distances[k][i];
            row = k;
            col = i;
          }
        }
      }
      group.clusters[col].id = prev_ids[row]; //assign the same id to the clusters with minimum (mean) phi distance
      corresp_matrix[prev_ids[row]] = col; //add correspondence of previous ID to current INDEX
      //invalidate selected row and col by assigning distance=1000
      for (int k = 0; k < prev_cluster_num; k++)
        phi_distances[k][col] = 1000;
      phi_distances[row].assign(cluster_num, 1000);
    }

    if (cluster_num > prev_cluster_num)
    {
      uint j = 0; //for the rest of the clusters assign appropriate id
      std::vector<uint>::iterator it;
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
          cl.id = j;
          prev_ids.push_back(j);
        }
      }
    }
    prev_ids.clear();
    prev_mean_phi.clear();
    prev_cluster_num = cluster_num;
    prev_mean_phi = cur_mean_phi;
    for (int i = 0; i < cluster_num; i++)
      prev_ids.push_back(group.clusters[i].id);
  }

  void doClustering(const dwal_planner::Sampled_Cluster::ConstPtr& msg)
  {
    //msg contains all sampled paths, for the given level, up to their collision point (if one exists)
    cluster_num = 0;
    Rcl = levels[level_index];
    path_num = msg->paths.size();
    free_path_indexes.clear();
    cluster_indexes.clear();
    cluster_vecs.clear();
    group.clusters.clear();
    for (int k = 0; k < marker_num; k++)
      path_array_marker.markers[k].points.clear(); //clear all path markers

    for (int k = 0; k < path_num; k++)
    { //get free paths
      if (msg->paths[k].colission_R > Rcl)
        free_path_indexes.push_back(k);
    }

    if (!free_path_indexes.size()) //do we have at least 1 valid path?
    {
      group_publisher.publish(group); //publish empty clusters
      markers_publisher.publish(path_array_marker); //publish empty markers and exit
      return;
    }

    // do cluster slicing with source group, if flag is raised
    //////--- start slicing -----------------------------------------------------------
    if (do_slice && source_slice_group_ptr->clusters.size() != 0)
    {
//      ROS_INFO("--do slicing");
      source_slice_cluster_index = clusterIdtoIndex(source_slice_cluster_id, source_slice_group_ptr);
      if (source_slice_cluster_index == -1)
      {
        ROS_INFO("--- Source cluster ID not found. Skipping slicing...");
        return;
      }
      switch (direction)
      {
        case false: //false = right
          max_curv_slice = source_slice_group_ptr->clusters[source_slice_cluster_index].paths.back().curvature;
          if (source_slice_cluster_index > 0)
            min_curv_slice = source_slice_group_ptr->clusters[source_slice_cluster_index - 1].paths.back().curvature;
          else
            min_curv_slice = msg->paths.front().curvature;
          break;

        case true: //true = left
          min_curv_slice = source_slice_group_ptr->clusters[source_slice_cluster_index].paths[0].curvature;
          if (source_slice_cluster_index < source_slice_group_ptr->clusters.size() - 1)
            max_curv_slice = source_slice_group_ptr->clusters[source_slice_cluster_index + 1].paths[0].curvature;
          else
            max_curv_slice = msg->paths.back().curvature;
          break;
      }

      for (int &k : free_path_indexes)
        if (msg->paths[k].curvature >= min_curv_slice && msg->paths[k].curvature <= max_curv_slice)
          sliced_path_indexes.push_back(k);

      free_path_indexes = sliced_path_indexes;
      sliced_path_indexes.clear();
    }
    //////--- end slicing ----------------------------------------------------------------

    //////--- start clustering -----------------------------------------------------------
    cluster_indexes.push_back(free_path_indexes[0]); //1st path is always a cluster border
    phi0 = cluster_lib::curv2phi(msg->paths[free_path_indexes[0]].curvature, Rcl);
    for (int k = 1; k < (int)free_path_indexes.size(); k++)
    {
      Dind = free_path_indexes[k] - cluster_indexes.back();
      if (Dind > cluster_separation) //gap found
      {
        Dphip = cluster_lib::curv2phi(msg->paths[cluster_indexes.back()].curvature, Rcl) - phi0;
        if (cluster_lib::clusterChord(Rcl, Dphip) >= min_cluster_span) //is last cluster big enough?
          cluster_vecs.push_back(cluster_indexes); //push it to cluster vecs
        cluster_indexes.clear(); //clear to begin new cluster
        phi0 = cluster_lib::curv2phi(msg->paths[free_path_indexes[k]].curvature, Rcl);
        cluster_indexes.push_back(free_path_indexes[k]); //add current path
      }
      else if (Dind <= 2)
        cluster_indexes.push_back(free_path_indexes[k]); //add current path
    }

    Dphip = cluster_lib::curv2phi(msg->paths[free_path_indexes.back()].curvature, Rcl) - phi0;
    if (cluster_lib::clusterChord(Rcl, Dphip) >= min_cluster_span) //check last curv
      cluster_vecs.push_back(cluster_indexes);

    cluster_num = (int)cluster_vecs.size();
    //////--- end clustering -----------------------------------------------------------

    //populate cluster_group msg and path markers
    int j;
    for (std::vector<int> &clvec : cluster_vecs)
    {
      tmp_cluster.paths.clear(); //clear temp cluster
      tmp_cluster.id = 100; //start with id=100. Will change in correspondence matching later
      tmp_cluster.R = Rcl;
      for (int &l : clvec)
      {
        j = 0;
        if (*spin != 0)
        {
          for (dwal_planner::Pose2D_32 const &p : msg->paths[l].poses)
          {
            p0.x = p.x;
            p0.y = p.y;
            path_array_marker.markers[l].points.push_back(p0);
            if (j >= msg->paths[l].level_inds[level_index])
              break;
            j++;
          }
        }
        tmp_path.curvature = msg->paths[l].curvature;
        tmp_path.phi = cluster_lib::curv2phi(msg->paths[l].curvature, Rcl);
        tmp_path.cost = msg->paths[l].costs[level_index];
        tmp_cluster.paths.push_back(tmp_path); //add path to current cluster
      }
      group.clusters.push_back(tmp_cluster);
    }
    findCorrespondence(); //set cluster ids according to correspondence matching

    if (*spin != 0)
    {
      //draw clusters with different color
      for (int k = 0; k < cluster_num; k++)
        for (int l : cluster_vecs[k])
          setColor(path_array_marker.markers[l], group.clusters[k].id,
                   0.3 + 0.7 * ((double)msg->paths[l].costs[level_index]) / 128.0);
    }

    //publish markers & groups
    group_publisher.publish(group);
    markers_publisher.publish(path_array_marker);
//    if(do_slice==true && group.clusters.size()==0)
//    {
//      ROS_INFO("----->empty clusters");
//    }
  }
};

/* main class that handles the sampled paths. It creates Clustering objects for each level (group)
 and performs the clustering by calling their appropriate method */

class ControlClass
{
public:
  std::vector<int> spin, clustering_order;
  ros::NodeHandle nh;
  ros::Subscriber sampled_cluster_subscriber;
  ros::ServiceServer toggleSpinService, toggleSliceService;
  std::vector<Clustering> group_objects;
  double Hz;
  bool do_slice;
  unsigned int group_num;
  std::vector<std::string> postfixes;
  std::vector<double> levels;
  std::string markerTopic, clusterTopic;

  bool spinSrvHandler(dwal_planner::toggleClusters::Request & req, dwal_planner::toggleClusters::Response & res)
  {
    for (unsigned int k = 0; k < req.flags.size(); k++)
      spin[k] = req.flags[k];
    res.success = true;
    return true;
  }

  bool sliceSrvHandler(dwal_planner::toggleSlice::Request & req, dwal_planner::toggleSlice::Response & res)
  {
    clustering_order.clear();
    clustering_order.push_back(req.source_group_index);
    clustering_order.push_back(req.target_group_index);
    for (int k = 0; k < (int)group_num; k++)
    {
      group_objects[k].do_slice = false; //reset slicing flags to all groups
      do_slice = false; //reset slicing flag to control class
      //rearrange clustering calling order to ensure source ptr is always valid
      if ((k != req.source_group_index) && (k != req.target_group_index))
        clustering_order.push_back(k);
    }

    if (req.do_slice)
    {
      group_objects[req.target_group_index].do_slice = true; //set slicing flag to target group
      group_objects[req.target_group_index].direction = req.direction; //set slicing direction to target group
      group_objects[req.target_group_index].source_slice_cluster_id = req.source_cluster_id;
      do_slice = true;
//      ROS_INFO("--start slicing");
    }
//    else
//      ROS_INFO("--stop slicing");
    res.success = true;
    return true;
  }

  void sampledClusterHandler(const dwal_planner::Sampled_Cluster::ConstPtr& msg)
  {
    if (do_slice)
    {
      Clustering & src_cluster = group_objects[clustering_order[0]];
      Clustering & trg_cluster = group_objects[clustering_order[1]];

      src_cluster.doClustering(msg); //do src group clustering
      trg_cluster.source_slice_group_ptr = &src_cluster.group;
      // check to see if prev src frame was empty
      if (src_cluster.corresp_matrix.size())
      {
        trg_cluster.source_slice_cluster_index = src_cluster.corresp_matrix[trg_cluster.source_slice_cluster_id];
        trg_cluster.source_slice_cluster_id = src_cluster.group.clusters[trg_cluster.source_slice_cluster_index].id;
      }
      else
        trg_cluster.source_slice_cluster_index = -1;
      trg_cluster.doClustering(msg); //do target group clustering
      for (unsigned int k = 2; k < group_num; k++)
        group_objects[clustering_order[k]].doClustering(msg);
    }
    else
    {
      for (unsigned int k = 0; k < group_num; k++)
        group_objects[clustering_order[k]].doClustering(msg);
    }

  }

  ControlClass()
  {
    markerTopic = "cluster_markers_";
    clusterTopic = "clusters_";
    nh.getParam("dwal_clustering/postfix", postfixes);
    nh.getParam("dwal_clustering/levels", levels);
    nh.param("dwal_generator/Hz", Hz, 5.0);
    nh.getParam("dwal_clustering/spin", spin);

    group_num = postfixes.size();
    if (group_num != levels.size())
    {
      ROS_INFO(" \n\n----ERROR : 'postfix' and 'levels' should have the same number of elements...\n"
               "----aborting launch of clustering node...\n\n");
      ros::shutdown();
      return;
    }

    do_slice = false;
    group_objects.resize(group_num);
    toggleSpinService = nh.advertiseService("toggle_cluster_spin", &ControlClass::spinSrvHandler, this);
    toggleSliceService = nh.advertiseService("toggle_cluster_slice", &ControlClass::sliceSrvHandler, this);
    sampled_cluster_subscriber = nh.subscribe<dwal_planner::Sampled_Cluster>("sampled_paths", 1,
                                                                          &ControlClass::sampledClusterHandler, this);

    for (unsigned int k = 0; k < group_num; k++)
    {
      clustering_order.push_back(k);
      group_objects[k].level_index = k;
      group_objects[k].spin = spin.begin() + k;
      group_objects[k].levels = levels;

      group_objects[k].markers_publisher = group_objects[k].nh.advertise<visualization_msgs::MarkerArray>(
          markerTopic + postfixes[k], 2);

      group_objects[k].group_publisher = group_objects[k].nh.advertise<dwal_planner::Cluster_Group>(
          clusterTopic + postfixes[k], 2);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwal_clustering");

  ControlClass ctrlObj;
  ros::Rate r(ctrlObj.Hz);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return (0);

}
