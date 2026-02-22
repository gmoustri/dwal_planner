#include <dwal_planner/libdwal_cluster.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <dwal_planner/msg/sampled_cluster.hpp>
#include <dwal_planner/msg/sampled_path.hpp>
#include <dwal_planner/msg/pose2_d32.hpp>

class TrajectoryGenerator
{
public:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<dwal_planner::msg::SampledCluster>::SharedPtr sampledCl_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampledCl_markers_publisher;

  std::shared_ptr<rclcpp::Node> node_;
  std::string odom_Topic;

  std::unique_ptr<tf2_ros::Buffer> tfBuff;
  std::unique_ptr<tf2_ros::TransformListener> tfl;
  costmap_2d::Costmap2DROS *costmap;
  base_local_planner::CostmapModel *cmap_model;

  visualization_msgs::msg::Marker path_marker;
  visualization_msgs::msg::MarkerArray cluster_markers;
  geometry_msgs::msg::Point p0;

  double sim_period{}, DS{}, alpha{}, kmax{}, Hz{}, Rmax{}, Dphi{}, phi0{};
  std::vector<double> pos, vel, levels;

  cluster_lib::SimpleTrajectoryGenerator tp;
  base_local_planner::LocalPlannerLimits alims;
  std::vector<dwal_planner::msg::SampledPath> paths;
  int marker_num{0}, path_num{0};
  dwal_planner::msg::SampledCluster cluster;

  explicit TrajectoryGenerator(const std::shared_ptr<rclcpp::Node>& node);
  ~TrajectoryGenerator();

  void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos[0] = msg->pose.pose.position.x;
    pos[1] = msg->pose.pose.position.y;
    pos[2] = tf2::getYaw(msg->pose.pose.orientation);
    
    double v_,w_;
    v_=std::fabs(msg->twist.twist.linear.x); //enforce forward motion
    w_=msg->twist.twist.angular.z ;

    //enforce bounds
    vel[0] = abs(v_) > alims.max_vel_trans ? sgn(v_)*alims.max_vel_trans : v_;
    vel[1] = abs(w_) > alims.max_vel_theta ? sgn(w_)*alims.max_vel_theta : w_;

    path_num = tp.initialise(pos, vel, Rmax);

    if (!costmap || !costmap->getCostmap()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "OccupancyGrid costmap not ready yet; skipping this cycle");
      return;
    }
    // tp.getTrajectories(paths, costmap);
    tp.getTrajectoriesWithFootprint(paths, costmap, cmap_model);

    cluster.paths.clear();
    cluster.pose0.clear();

    for (int k = 0; k < path_num; k++)
      cluster.paths.push_back(paths[static_cast<size_t>(k)]);
    cluster.pose0 = pos;

    sampledCl_publisher->publish(cluster);

    for (int k = 0; k < marker_num; k++)
      cluster_markers.markers[static_cast<size_t>(k)].points.clear();

    for (int l = 0; l < path_num; l++)
    {
      for (const dwal_planner::msg::Pose2D32 &p : cluster.paths[static_cast<size_t>(l)].poses)
      {
        p0.x = p.x;
        p0.y = p.y;
        cluster_markers.markers[static_cast<size_t>(l)].points.push_back(p0);
      }
    }
    sampledCl_markers_publisher->publish(cluster_markers);
  }
};

TrajectoryGenerator::TrajectoryGenerator(const std::shared_ptr<rclcpp::Node>& node)
: node_(node), costmap(nullptr), cmap_model(nullptr)
{
  tfBuff = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tfBuff->setUsingDedicatedThread(true);
  tfl = std::make_unique<tf2_ros::TransformListener>(*tfBuff);

  std::string occ_topic_default = "/occupancy_map_local";
  node_->declare_parameter<std::string>("occ_topic", occ_topic_default);
  std::string occ_topic = occ_topic_default;
  node_->get_parameter("occ_topic", occ_topic);

  node_->declare_parameter<std::vector<double>>("footprint", {});
  std::vector<double> footprint_xy;
  node_->get_parameter("footprint", footprint_xy);

  try {
    tfBuff->lookupTransform("base_link", "odom", tf2::TimePointZero, std::chrono::seconds(3));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  costmap    = new costmap_2d::Costmap2DROS(node_, occ_topic);
  auto cm = costmap->getCostmap();
  while (rclcpp::ok() && (!cm || cm->getSizeInCellsX() == 0 || cm->getSizeInCellsY() == 0)) {
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Waiting for OccupancyGrid on '%s' to populate...", occ_topic.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(node_);
    cm = costmap->getCostmap();
  }
  cmap_model = new base_local_planner::CostmapModel(costmap->getCostmap());

  std::vector<geometry_msgs::msg::Point> fp;
  if (footprint_xy.size() >= 6 && footprint_xy.size() % 2 == 0) {
    fp.reserve(footprint_xy.size()/2);
    for (size_t i = 0; i < footprint_xy.size(); i += 2) {
      geometry_msgs::msg::Point p;
      p.x = footprint_xy[i]; p.y = footprint_xy[i+1]; p.z = 0.0;
      fp.push_back(p);
    }
  }
  else {
    RCLCPP_FATAL(node_->get_logger(),
                "Footprint parameter malformed or too short; using default footprint");
    throw(std::runtime_error("Footprint parameter malformed or too short"));
  }
  costmap->setRobotFootprint(fp);

  pos.resize(3);
  vel.resize(2);

  node_->declare_parameter<double>("dwal_generator/acc_lim_x", 0.5);
  node_->declare_parameter<double>("dwal_generator/acc_lim_th", 0.5);

  node_->declare_parameter<double>("dwal_generator/max_trans_vel", 0.5);
  node_->declare_parameter<double>("dwal_generator/min_trans_vel", 0.1);
  node_->declare_parameter<double>("dwal_generator/max_vel_theta", 1.0);

  node_->declare_parameter<double>("dwal_generator/sim_period", 0.2);
  node_->declare_parameter<double>("dwal_generator/DS", 0.1);
  node_->declare_parameter<double>("dwal_generator/alpha", 0.2);
  node_->declare_parameter<double>("dwal_generator/Kmax", 2.0);
  node_->declare_parameter<double>("dwal_generator/Hz", 5.0);
  node_->declare_parameter<std::vector<double>>("dwal_clustering/levels", {});
  node_->declare_parameter<std::string>("odometryTopic", "/odom");

  double acc_lim_x{0.5}, acc_lim_th{0.5};
  node_->get_parameter("dwal_generator/acc_lim_x",      acc_lim_x);
  node_->get_parameter("dwal_generator/acc_lim_th",     acc_lim_th);

  node_->get_parameter("dwal_generator/max_trans_vel",  alims.max_vel_trans);
  node_->get_parameter("dwal_generator/min_trans_vel",  alims.min_vel_trans);
  node_->get_parameter("dwal_generator/max_vel_theta",  alims.max_vel_theta);

  node_->get_parameter("dwal_generator/sim_period", sim_period);
  node_->get_parameter("dwal_generator/DS",         DS);
  node_->get_parameter("dwal_generator/alpha", alpha);
  node_->get_parameter("dwal_generator/Kmax",       kmax);
  node_->get_parameter("dwal_generator/Hz",         Hz);
  node_->get_parameter("dwal_clustering/levels",    levels);

  node_->get_parameter("odometryTopic", odom_Topic);

  alims.setAccLimits(static_cast<float>(acc_lim_x), 0.0f, static_cast<float>(acc_lim_th));

  if (alims.min_vel_trans <= 0.0)
  {
    RCLCPP_INFO(node_->get_logger(), "---> min_vel_trans must be >0. Aborting...");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(node_->get_logger(), " --> max_vel_trans = %f", alims.max_vel_trans);
  RCLCPP_INFO(node_->get_logger(), " --> min_vel_trans = %f", alims.min_vel_trans);
  RCLCPP_INFO(node_->get_logger(), " --> max_vel_theta = %f", alims.max_vel_theta);

  Rmax = levels.back();

  Dphi = 2 * std::asin(alpha / (2 * Rmax));
  if (std::fabs(kmax) < (M_SQRT2 / Rmax))
    phi0 = -std::fabs(std::asin(0.5 * Rmax * kmax));
  else
    phi0 = -std::fabs(std::acos(1 / (Rmax * kmax)));

  marker_num = static_cast<int>(2 * std::fabs(phi0) / Dphi + 1);
  RCLCPP_INFO(node_->get_logger(), "----marker_num=  %d", marker_num);

  auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/dwal_planner/dwal_clustering");
  client->set_parameters({ rclcpp::Parameter("dwal_clustering/Marker_num", marker_num) });

  paths.reserve(static_cast<size_t>(marker_num));
  for (int k = 0; k < marker_num; k++)
    paths.emplace_back();

  tp.setParameters(sim_period, DS, alpha, kmax, Dphi, &alims, levels);

  cluster.paths.clear();
  cluster.pose0.resize(3);
  cluster.levels = levels;

  path_marker.header.frame_id = "odom";
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
  path_marker.color.g = 0.7;
  path_marker.color.b = 0.7;
  path_marker.color.a = 0.8;
  for (int k = 0; k < marker_num; k++)
  {
    path_marker.id = k;
    cluster_markers.markers.push_back(path_marker);
  }
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  delete cmap_model;
  delete costmap;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dwal_generator");

  TrajectoryGenerator generator(node);

  std::cout << generator.odom_Topic << "\n";
  generator.sampledCl_publisher = node->create_publisher<dwal_planner::msg::SampledCluster>("sampled_paths", rclcpp::QoS(2));
  generator.sampledCl_markers_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("sampled_pathMarkers", rclcpp::QoS(2));
  generator.odom_subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
    generator.odom_Topic, rclcpp::QoS(1),
    std::bind(&TrajectoryGenerator::odomHandler, &generator, std::placeholders::_1));

  double hz = generator.Hz;
  node->get_parameter("dwal_generator/Hz", hz);
  rclcpp::Rate r(hz);

  rclcpp::Time t0 = node->get_clock()->now();
  rclcpp::Time tini =t0;

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    r.sleep();
    if(node->get_clock()->now()-t0 > rclcpp::Duration(2,0))
    {
      t0 = node->get_clock()->now();
      RCLCPP_INFO(node->get_logger(), "Spinning: T= %f passed", (t0-tini).seconds());
    }
  }

  rclcpp::shutdown();
  return 0;
}
