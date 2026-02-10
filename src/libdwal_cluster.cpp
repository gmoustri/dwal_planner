/*********************************************************************
 Mod of simple_trajectory_generator. Uses only DWA and exposes input space samples
 *********************************************************************/
#include <dwal_planner/libdwal_cluster.h>  

#include <rclcpp/rclcpp.hpp>

int sgn(double val)
{
  return (int)(0.0 < val) - (val < 0.0);
}

double sat(double in, double lim_up, double lim_down)
{
  if (in > lim_up)
    return (lim_up);
  else if (in < lim_down)
    return (lim_down);
  else
    return in;
}

double rate_limiter(double d_value, double rate_lim, double dt)
{
  double a = d_value / dt;
  if (std::fabs(a) <= rate_lim)
    return d_value;
  else
    return rate_lim * dt * sgn(a);
}

namespace cluster_lib
{

double phi2curv(double phi, double R)
{
  if (std::fabs(phi) < M_PI_4)
    return 2 * std::sin(phi) / R;
  else
    return sgn(phi) / (R * std::cos(phi));
}

double curv2phi(double k, double R)
{
  if (std::fabs(k) < (M_SQRT2 / R))
    return std::asin(0.5 * R * k);
  else
    return sgn(k) * std::acos(1 / (std::fabs(k) * R));
}

double clusterChord(double Rcirc, double DPhi)
{
  return Rcirc * 2 * std::sin(0.5 * std::fabs(DPhi));
}

void generatePath(nav_msgs::msg::Path &path_out, double curv, double x0, double y0, double th0, double DS, double Rc2)
{
  // trajectory might be reused so we'll make sure to reset it
  path_out.header.stamp = rclcpp::Clock().now();
  path_out.poses.clear();

  double A, sincA, x, y, theta, dxLin, dyLin, Rp2;
  double dth = curv * DS;
  double S = 0;
  A = dth / 2;
  if (curv == 0)
    sincA = DS;
  else
    sincA = 2 * std::sin(A) / curv;

  geometry_msgs::msg::PoseStamped Posetmp;

  x = x0;
  y = y0;
  theta = th0;

  dxLin = sgn(curv) * DS * -std::sin(th0); // cos(theta0+M_PI_2);
  dyLin = sgn(curv) * DS *  std::cos(th0); // sin(theta0+M_PI_2);

  // add initial point to the trajectory -- assume it's always free
  Posetmp.pose.position.x = x;
  Posetmp.pose.position.y = y;

  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    Posetmp.pose.orientation = tf2::toMsg(q);
  }

  path_out.poses.push_back(Posetmp);
  Rp2 = 0;

  // generate path
  while (Rp2 < Rc2)
  {
    S += DS;
    if (std::fabs(theta - th0) < M_PI_2)
    {
      x += sincA * std::cos(theta + A);
      y += sincA * std::sin(theta + A);
      theta += dth;
    }
    else
    {
      x += dxLin;
      y += dyLin;
    }
    Rp2 = (x - x0) * (x - x0) + (y - y0) * (y - y0);

    Posetmp.pose.position.x = x;
    Posetmp.pose.position.y = y;

    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta);
      Posetmp.pose.orientation = tf2::toMsg(q);
    }

    path_out.poses.push_back(Posetmp);
  } // end path generation steps
}

void SimpleTrajectoryGenerator::setParameters(double sim_period, double DS, double alpha, double kmax, double Dphi,
                                              base_local_planner::LocalPlannerLimits *limits,
                                              std::vector<double> &levels)
{
  sim_period_ = sim_period;
  DS_ = DS;
  alpha_ = alpha;
  limits_ = limits;
  acc_lim = limits->getAccLimits();
  kmax_ = kmax;
  Dphi_ = Dphi;
  levels_ = levels;
}

int SimpleTrajectoryGenerator::initialise(const std::vector<double> &pos, const std::vector<double> &vel, double Rcirc)
{
  /*
   * We actually generate all curvature sample vectors here, from which to generate trajectories later on
   */
  pos_ = pos;
  vel_ = vel;
  next_sample_index_ = 0;
  Rmax_ = Rcirc;
  Rmax2_ = Rcirc * Rcirc;

  // clear curvs_ & phis_ to reuse them
  curvs_.clear();
  phis_.clear();

  // compute the feasible velocity space based on the rate at which we run
  std::vector<double> max_vel(2);
  std::vector<double> min_vel(2);

  // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
  max_vel[0] = std::min(limits_->max_vel_trans, vel[0] + acc_lim[0] * sim_period_);
  min_vel[0] = std::max(limits_->min_vel_trans, vel[0] - acc_lim[0] * sim_period_);

  max_vel[1] = std::min(limits_->max_vel_theta, vel[1] + acc_lim[2] * sim_period_);
  min_vel[1] = std::max(-limits_->max_vel_theta, vel[1] - acc_lim[2] * sim_period_);

  if (max_vel[1]>=0)
    maxCurv = std::clamp(max_vel[1] / min_vel[0], -kmax_, kmax_);  // max curvature = max w / min v
  else
    maxCurv = std::clamp(max_vel[1] / max_vel[0], -kmax_, kmax_);  // max curvature = max w / max v
  
  if (min_vel[1]<=0)
    minCurv = std::clamp( min_vel[1] / min_vel[0],-kmax_, kmax_); // min curvature = min w / min v
  else
    minCurv = std::clamp( min_vel[1] / max_vel[0], -kmax_, kmax_); // min curvature = min w / max v
  

  // create vector with sample curvatures
  double phi0, phin;
  phi0 = curv2phi(minCurv, Rmax_);
  phin = curv2phi(maxCurv, Rmax_);

  Curv_num_ = static_cast<int>((phin - phi0) / Dphi_) + 1;

  double phi = phi0;
  double kk;
  for (int k = 0; k < Curv_num_; k++)
  {
    if (std::fabs(phi) < M_PI_4)
      kk = 2 * std::sin(phi) / Rmax_;
    else
      kk = sgn(phi) / (Rmax_ * std::cos(phi));
    curvs_.push_back(kk);
    phis_.push_back(phi);
    phi += Dphi_;
  }
  return Curv_num_;
}

/**
 * Create and return the next sample trajectory
 */
int SimpleTrajectoryGenerator::getTrajectories(std::vector<dwal_planner::msg::SampledPath> &trajs,
                                               costmap_2d::Costmap2DROS *costmap)
{
  int result = 0;
  // update costmap just before checking trajectories
  cmap = costmap->getCostmap();
  // loop over trajectories and sample them
  for (int k = 0; k < Curv_num_; k++)
  {
    result += generateTrajectory(curvs_[next_sample_index_], trajs[next_sample_index_], costmap);
    next_sample_index_++;
  }
  return result;
}

int SimpleTrajectoryGenerator::generateTrajectory(double sample_curv, dwal_planner::msg::SampledPath &traj,
                                                  costmap_2d::Costmap2DROS *costmap)
{
  // trajectory might be reused so we'll make sure to reset it
  traj.poses.clear();
  traj.costs.clear();      // clear costs
  traj.level_inds.clear();

  traj.curvature = sample_curv; // xv is the current curvature
  traj.colission_r = 10000;    

  unsigned int mx, my;
  unsigned int curlevel = 0;
  double phi, dphi, A, x, y, theta, dxLin, dyLin, Rp2;
  int cellCost;
  bool stop = false;
  bool jumped = false;
  dwal_planner::msg::Pose2D32 pose;

  A = 2 / sample_curv;
  dphi = sample_curv * DS_ / 2;
  phi = 0; x = 0; y = 0; theta = 0;

  dxLin = DS_ * std::cos(pos_[2] + sgn(sample_curv) * M_PI_2);
  dyLin = DS_ * std::sin(pos_[2] + sgn(sample_curv) * M_PI_2);

  // add initial point to the trajectory -- assume its always free
  pose.x = static_cast<float>(pos_[0]);
  pose.y = static_cast<float>(pos_[1]);
  pose.theta = static_cast<float>(pos_[2]);
  traj.poses.push_back(pose);
  Rp2 = 0;
  traj.costs.push_back(0);

  // simulate the trajectory and check for collisions, updating costs along the way
  while (!stop)
  {
    // try step increase in path
    if (std::fabs(phi) < M_PI_4)
    {
      phi += dphi;
      x = A * std::sin(phi) * std::cos(pos_[2] + phi) + pos_[0];
      y = A * std::sin(phi) * std::sin(pos_[2] + phi) + pos_[1];
      theta = 2 * phi + pos_[2];
    }
    else
    {
      x += dxLin;
      y += dyLin;
    }
    Rp2 = (x - pos_[0]) * (x - pos_[0]) + (y - pos_[1]) * (y - pos_[1]);

    // check if it crosses a level and fix it
    if (Rp2 > (levels_[curlevel] * levels_[curlevel]))
    {
      phi = curv2phi(sample_curv, levels_[curlevel]);
      x = levels_[curlevel] * std::cos(pos_[2] + phi) + pos_[0];
      y = levels_[curlevel] * std::sin(pos_[2] + phi) + pos_[1];
      theta = 2 * phi + pos_[2];
      Rp2 = levels_[curlevel] * levels_[curlevel];
      curlevel++;
      jumped = true;
    }

    costmap->getCostmap()->worldToMap(x, y, mx, my);
    cellCost = static_cast<int>(costmap->getCostmap()->getCost(mx, my));
    if (cellCost >= 128)
    {
      traj.costs.back() = 255; // trajectory collides.
      traj.colission_r = std::sqrt(Rp2);
      stop = true;
    }
    else
    { // update cost, add point and continue
      traj.costs.back() = cellCost > traj.costs.back() ? cellCost : traj.costs.back();
      pose.x = static_cast<float>(x);
      pose.y = static_cast<float>(y);
      pose.theta = static_cast<float>(theta);
      traj.poses.push_back(pose);
      if (jumped)
      {
        traj.level_inds.push_back(static_cast<int>(traj.poses.size()) - 1);
        traj.costs.push_back(traj.costs.back());
        jumped = false;
      }
      if (curlevel >= levels_.size())
        stop = true;
    }
  } // end for simulation steps
  return 1; // trajectory does not collide (until last step)
}

int SimpleTrajectoryGenerator::getTrajectoriesWithFootprint(std::vector<dwal_planner::msg::SampledPath> &trajs,
                                                costmap_2d::Costmap2DROS *costmap,
                                                base_local_planner::CostmapModel *cmap_model)
{
  int result = 0;
  // update costmap just before checking trajectories
  cmap = costmap->getCostmap();
  // loop over trajectories and sample them
  for (int k = 0; k < Curv_num_; k++)
  {
    result += generateTrajectoryWithFootprint(curvs_[next_sample_index_], trajs[next_sample_index_], costmap, cmap_model);
    next_sample_index_++;
  }
  return result;
}

int SimpleTrajectoryGenerator::generateTrajectoryWithFootprint(double sample_curv, dwal_planner::msg::SampledPath &traj,
                                                   costmap_2d::Costmap2DROS *costmap,
                                                   base_local_planner::CostmapModel *cmap_model)
{
  // Generate trajectory using polygon footprint for collision detection
  // trajectory might be reused so we'll make sure to reset it
  traj.poses.clear();
  traj.costs.clear(); // clear costs
  traj.level_inds.clear();

  traj.curvature = sample_curv; // xv is the current curvature
  traj.colission_r = 10000;     

  unsigned int curlevel = 0;
  double phi, dphi, A, x, y, theta, dxLin, dyLin, Rp2;
  int footprintCost;
  bool stop = false;
  bool jumped = false;
  dwal_planner::msg::Pose2D32 pose;

  A = 2 / sample_curv;
  dphi = sample_curv * DS_ / 2;
  phi = 0; x = 0; y = 0; theta = 0;

  dxLin = DS_ * std::cos(pos_[2] + sgn(sample_curv) * M_PI_2);
  dyLin = DS_ * std::sin(pos_[2] + sgn(sample_curv) * M_PI_2);

  // add initial point to the trajectory -- assume its always free
  pose.x = static_cast<float>(pos_[0]);
  pose.y = static_cast<float>(pos_[1]);
  pose.theta = static_cast<float>(pos_[2]);
  traj.poses.push_back(pose);
  Rp2 = 0;
  traj.costs.push_back(0);

  // simulate the trajectory and check for collisions, updating costs along the way
  while (!stop)
  {
    // try step increase in path
    if (std::fabs(phi) < M_PI_4)
    {
      phi += dphi;
      x = A * std::sin(phi) * std::cos(pos_[2] + phi) + pos_[0];
      y = A * std::sin(phi) * std::sin(pos_[2] + phi) + pos_[1];
      theta = 2 * phi + pos_[2];
    }
    else
    {
      x += dxLin;
      y += dyLin;
    }
    Rp2 = (x - pos_[0]) * (x - pos_[0]) + (y - pos_[1]) * (y - pos_[1]);

    // check if it crosses a level and fix it
    if (Rp2 > (levels_[curlevel] * levels_[curlevel]))
    {
      phi = curv2phi(sample_curv, levels_[curlevel]);
      x = levels_[curlevel] * std::cos(pos_[2] + phi) + pos_[0];
      y = levels_[curlevel] * std::sin(pos_[2] + phi) + pos_[1];
      theta = 2 * phi + pos_[2];
      Rp2 = levels_[curlevel] * levels_[curlevel];
      curlevel++;
      jumped = true;
    }

    footprintCost = cmap_model->footprintCost(x, y, theta, costmap->getRobotFootprint());

    // if (footprintCost < 0 || footprintCost >= nav2_costmap_2d::LETHAL_OBSTACLE || footprintCost == nav2_costmap_2d::NO_INFORMATION)      
    if (0)      
    { 
      traj.costs.back() = 255; // trajectory collides.
      traj.colission_r = std::sqrt(Rp2);
      stop = true;
    }
    else
    {
      // update cost, add point and continue
      traj.costs.back() = footprintCost > traj.costs.back() ? footprintCost : traj.costs.back();
      pose.x = static_cast<float>(x);
      pose.y = static_cast<float>(y);
      pose.theta = static_cast<float>(theta);
      traj.poses.push_back(pose);
      if (jumped)
      {
        traj.level_inds.push_back(static_cast<int>(traj.poses.size()) - 1);
        traj.costs.push_back(traj.costs.back());
        jumped = false;
      }
      if (curlevel >= levels_.size())
        stop = true;
    }

  } // end for simulation steps
  return 1; // trajectory does not collide.
}

} // namespace cluster_lib
