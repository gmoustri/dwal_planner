#include <dwal_planner/libdwal_cluster.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dwal_planner/Sampled_Cluster.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/MarkerArray.h>

class TrajectoryGenerator
{
public:
  ros::Subscriber odom_subscriber;
  ros::Publisher sampledCl_publisher, sampledCl_markers_publisher;
  ros::NodeHandle nh;
  std::string odom_Topic;

  //tf::TransformListener *tf;
  tf2_ros::TransformListener *tfl;
  tf2_ros::Buffer *tfBuff;
  costmap_2d::Costmap2DROS *costmap;
  base_local_planner::CostmapModel *cmap_model;

  visualization_msgs::Marker path_marker;
  visualization_msgs::MarkerArray cluster_markers;
  geometry_msgs::Point p0;

  double sim_period, DS, alpha, kmax, Hz, Rmax, Dphi, phi0;
  std::vector<double> pos, vel, levels;

  cluster_lib::SimpleTrajectoryGenerator tp;
  base_local_planner::LocalPlannerLimits alims;
  std::vector<dwal_planner::Sampled_Path> paths;
  int marker_num, path_num;
  dwal_planner::Sampled_Cluster cluster;

  TrajectoryGenerator();

  void odomHandler(const nav_msgs::Odometry::ConstPtr& msg)
  {
    //copy odometry data
    pos[0] = msg->pose.pose.position.x;
    pos[1] = msg->pose.pose.position.y;
    pos[2] = tf::getYaw(msg->pose.pose.orientation);

    vel[0] = msg->twist.twist.linear.x;
    vel[1] = msg->twist.twist.angular.z;

    path_num = tp.initialise(pos, vel, Rmax);

    ///tp.getTrajectories(paths, costmap);

    tp.getTrajectories2(paths, costmap,cmap_model);

    //populate cluster msg with thin paths
    cluster.paths.clear(); //clear cluster
    cluster.pose0.clear();
    for (unsigned int k = 0; k < path_num; k++)
    {
      cluster.paths.push_back(paths[k]); //add path to current cluster
    }
    cluster.pose0 = pos;
    //publish cluster of generated paths
    sampledCl_publisher.publish(cluster);

    //Populate markers
    for (int k = 0; k < marker_num; k++)
      cluster_markers.markers[k].points.clear(); //clear all path markers
    for (unsigned int l = 0; l < path_num; l++)
    {
      for (dwal_planner::Pose2D_32 const &p : cluster.paths[l].poses)
      {
        p0.x = p.x;
        p0.y = p.y;
        cluster_markers.markers[l].points.push_back(p0);
      }
    }
    sampledCl_markers_publisher.publish(cluster_markers);
  }

};

TrajectoryGenerator::TrajectoryGenerator() // @suppress("Class members should be properly initialized")
{

 tfBuff= new tf2_ros::Buffer(ros::Duration(10),true);
 tfBuff->setUsingDedicatedThread(true);
 tfl=new tf2_ros::TransformListener(*tfBuff);

 /* Wait until the transform is available to be given to costmap */
 try{
   tfBuff->lookupTransform("base_link", "odom",ros::Time(0),ros::Duration(3.0));
 }
 catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   ros::Duration(1.0).sleep();
 }

  costmap = new costmap_2d::Costmap2DROS("dwal_cmap", *tfBuff);
  cmap_model= new base_local_planner::CostmapModel(*(costmap->getCostmap()));
  std::cout<<"ff\n";

  pos.resize(3);
  vel.resize(2);
  alims.acc_lim_y = 0;

  nh.param("dwal_generator/acc_lim_x", alims.acc_lim_x, 0.5);
  nh.param("dwal_generator/acc_lim_th", alims.acc_lim_theta, 0.5);

  nh.param("dwal_generator/max_trans_vel", alims.max_vel_trans, 0.5);
  nh.param("dwal_generator/min_trans_vel", alims.min_vel_trans, 0.1);
  nh.param("dwal_generator/max_vel_theta", alims.max_vel_theta, 1.0);

  nh.param("dwal_generator/sim_period", sim_period, 0.2);
  nh.param("dwal_generator/DS", DS, 0.1);
  nh.param("dwal_generator/dwal_cmap/resolution", alpha, 0.2);
  nh.param("dwal_generator/Kmax", kmax, 2.0);
  nh.param("dwal_generator/Hz", Hz, 5.0);
  nh.getParam("dwal_clustering/levels", levels);

  nh.getParam("odometryTopic", odom_Topic);

  if (alims.min_vel_trans <= 0)
  {
    ROS_INFO("---> min_vel_trans must be >0. Aborting...");
    ros::shutdown();
    return;
  }

  Rmax = levels.back();

  Dphi = 2 * asin(alpha / (2 * Rmax));
  if (fabs(kmax) < (M_SQRT2 / Rmax))
    phi0 = -fabs(asin(0.5 * Rmax * kmax));
  else
    phi0 = -fabs(acos(1 / (Rmax * kmax)));

  marker_num = (int)2 * fabs(phi0) / Dphi + 1; //maximum number of paths--remains fixed
  ROS_INFO("----marker_num=  %d", marker_num);

  nh.setParam("dwal_clustering/Marker_num", marker_num); //set marker_num to param server

  dwal_planner::Sampled_Path path;
  for (int k = 0; k < marker_num; k++)
    paths.push_back(path);

  tp.setParameters(sim_period, DS, alpha, kmax, Dphi, &alims, levels);

  cluster.paths.clear();
  cluster.pose0.resize(3);
  cluster.levels = levels;

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
  path_marker.color.g = 0.7;
  path_marker.color.b = 0.7;
  path_marker.color.a = 0.8;
  for (int k = 0; k < marker_num; k++)
  {
    path_marker.id = k;
    cluster_markers.markers.push_back(path_marker);
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dwal_generator");

  TrajectoryGenerator o;
  o.odom_subscriber = o.nh.subscribe<nav_msgs::Odometry>(o.odom_Topic, 1, &TrajectoryGenerator::odomHandler, &o);
  o.sampledCl_publisher = o.nh.advertise<dwal_planner::Sampled_Cluster>("sampled_paths", 2);
  o.sampledCl_markers_publisher = o.nh.advertise<visualization_msgs::MarkerArray>("sampled_pathMarkers", 2);

  ros::Rate r(o.Hz);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return (0);

}
