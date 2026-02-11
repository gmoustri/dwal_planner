#ifndef DWAL_CLUSTER_LIB_H_
#define DWAL_CLUSTER_LIB_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <dwal_planner/msg/sampled_path.hpp>
#include <dwal_planner/msg/pose2_d32.hpp>

namespace base_local_planner {
class LocalPlannerLimits {
public:
  double max_vel_trans{0.0};
  double min_vel_trans{0.0};
  double max_vel_theta{0.0};

  Eigen::Vector3f getAccLimits() const { return acc_lim_; }

  void setAccLimits(float ax, float ay, float atheta) { acc_lim_ = Eigen::Vector3f(ax, ay, atheta); }

private:
  Eigen::Vector3f acc_lim_{0.f, 0.f, 0.f};
};

class CostmapModel {
public:
  explicit CostmapModel(nav2_costmap_2d::Costmap2D* cm)
  : checker_(cm) {}

  int footprintCost(double x, double y, double theta,
                    const std::vector<geometry_msgs::msg::Point>& footprint)
  {
    const double cost = checker_.footprintCostAtPose(x, y, theta, footprint);
    return (cost < 0.0) ? -1 : static_cast<int>(cost);
  }

private:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker_;
};
} // namespace base_local_planner

namespace costmap_2d {

class Costmap2DROS {
public:
  explicit Costmap2DROS(
      const std::shared_ptr<rclcpp::Node>& node,
      const std::string& occupancy_grid_topic,
      const std::string& footprint_param_name = "footprint")
  : node_(node), footprint_param_name_(footprint_param_name)
  {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    occ_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      occupancy_grid_topic, qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        this->updateFromOccupancyGrid(*msg);
      });
  }

  nav2_costmap_2d::Costmap2D* getCostmap() { return &cm_; }

  // Return whatever footprint we have (possibly empty if no param provided)
  std::vector<geometry_msgs::msg::Point> getRobotFootprint() const { return footprint_; }

  // Footprint setter
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point>& fp) { footprint_ = fp; }

private:

  void updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg)
  {
    const unsigned int w = msg.info.width;
    const unsigned int h = msg.info.height;
    const double res = msg.info.resolution;
    const double ox  = msg.info.origin.position.x;
    const double oy  = msg.info.origin.position.y;

    cm_.resizeMap(w, h, res, ox, oy);

    // Convert [-1, 0..100] → costmap [0..255]
    const auto& src = msg.data;
    unsigned char* dst = cm_.getCharMap();
    for (size_t i = 0; i < src.size(); ++i) {
      const int v = src[i];
      if (v < 0) {
        dst[i] = nav2_costmap_2d::NO_INFORMATION;   // 255
      } else if (v >= 100) {
        dst[i] = nav2_costmap_2d::LETHAL_OBSTACLE;  // 254
      } else {
        dst[i] = static_cast<unsigned char>(((double)v *254.0)/100.0); // 0-99 maps to 0..251 due to integer casting 
      }
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string footprint_param_name_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_sub_;
  nav2_costmap_2d::Costmap2D cm_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

} // namespace costmap_2d
// --------------------------------------------------------------------


double sat(double in, double limup, double limdown);
double rate_limiter(double d_value, double rate_lim, double dt);
int sgn(double val);

namespace cluster_lib
{

void generatePath(nav_msgs::msg::Path &PathOut, double curv, double x0, double y0, double th0, double DS, double Rc2);
double phi2curv(double phi, double Rcirc);
double curv2phi(double k, double Rcirc);
double clusterChord(double Rcirc, double DPhi);

class SimpleTrajectoryGenerator
{
public:
  nav2_costmap_2d::Costmap2D *cmap{nullptr};

  SimpleTrajectoryGenerator() = default;
  ~SimpleTrajectoryGenerator() = default;

  /**
   * This function is to be called only when parameters change
   */
  void setParameters(double sim_period, double DS, double alpha, double Rmin, double Dphi,
                     base_local_planner::LocalPlannerLimits *limits, std::vector<double> &levels);

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   */
  int initialise(const std::vector<double> &pos, const std::vector<double> &vel, double Rcirc);

  /**
   * Whether this generator can create more trajectories
   */
  int getTrajectories(std::vector<dwal_planner::msg::SampledPath> &path, costmap_2d::Costmap2DROS *costmap);
  int generateTrajectory(double sample_curv, dwal_planner::msg::SampledPath &path, costmap_2d::Costmap2DROS *costmap);

  int getTrajectoriesWithFootprint(std::vector<dwal_planner::msg::SampledPath> &path, costmap_2d::Costmap2DROS *costmap,
                       base_local_planner::CostmapModel *cmap_model);
  int generateTrajectoryWithFootprint(double sample_curv, dwal_planner::msg::SampledPath &path, costmap_2d::Costmap2DROS *costmap,
                          base_local_planner::CostmapModel *cmap_model);

protected:
  unsigned int next_sample_index_{0};
  base_local_planner::LocalPlannerLimits *limits_{nullptr};
  std::vector<double> pos_, vel_;
  double sim_period_{0.0}, DS_{0.0}, Smax_{0.0}, Thmax_{0.0}, alpha_{0.0}, kmax_{0.0},
         Rmax_{0.0}, Rmax2_{0.0}, Dphi_{0.0};
  int Curv_num_{0};
  Eigen::Vector3f acc_lim{0.f, 0.f, 0.f};
  double minCurv{0.0}, maxCurv{0.0};
  std::vector<double> curvs_, phis_, levels_;
};

} /* namespace cluster_lib */

#endif /* DWAL_CLUSTER_LIB_H_ */
