#ifndef DWAL_CLUSTER_LIB_H_
#define DWAL_CLUSTER_LIB_H_

#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <eigen3/Eigen/Core>
#include <dwal_planner/Sampled_Path.h>
#include <nav_msgs/Path.h>
#include "tf/tf.h"
#include <tf/transform_datatypes.h>

double sat(double in, double limup, double limdown);
double rate_limiter(double d_value, double rate_lim, double dt);
int sgn(double val);

namespace cluster_lib
{

void generatePath(nav_msgs::Path &PathOut, double curv, double x0, double y0, double th0, double DS, double Rc2);
double phi2curv(double phi, double Rcirc);
double curv2phi(double k, double Rcirc);
double clusterChord(double Rcirc, double DPhi);

class SimpleTrajectoryGenerator
{
public:
  costmap_2d::Costmap2D *cmap;

  SimpleTrajectoryGenerator()
  {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGenerator()
  {
  }

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
  int getTrajectories(std::vector<dwal_planner::Sampled_Path> &path, costmap_2d::Costmap2DROS *costmap);
  int generateTrajectory(double sample_curv, dwal_planner::Sampled_Path &path, costmap_2d::Costmap2DROS *costmap);

  int getTrajectories2(std::vector<dwal_planner::Sampled_Path> &path, costmap_2d::Costmap2DROS *costmap,
                       base_local_planner::CostmapModel *cmap_model);
  int generateTrajectory2(double sample_curv, dwal_planner::Sampled_Path &path, costmap_2d::Costmap2DROS *costmap,
                          base_local_planner::CostmapModel *cmap_model);

protected:

  unsigned int next_sample_index_;
  base_local_planner::LocalPlannerLimits *limits_;
  std::vector<double> pos_, vel_;
  double sim_period_, DS_, Smax_, Thmax_, alpha_, kmax_, Rmax_, Rmax2_, Dphi_;
  int Curv_num_;
  Eigen::Vector3f acc_lim;
  double minCurv, maxCurv;
  std::vector<double> curvs_, phis_, levels_;

};

} /* cluster_lib */

#endif /* FF_CLUSTER_LIB_H_ */
