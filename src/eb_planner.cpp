#include <autoware_msgs/Waypoint.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/utils.h>
// #include <tf/transform_datatypes.h>

// headers in Eigen
// #include <Eigen/Dense>
#include <distance_transform/distance_transform.hpp>
#include <grid_map_core/GridMap.hpp>
#include <qpOASES.hpp>
#include <qpOASES/QProblem.hpp>
// #include <qpoases_vendor/qpOASES.hpp>

#include <memory>

#include "workspace.h"
#include "osqp.h"

#include "reference_path.h"

#include "eb_planner.h"

QPPlanner::QPPlanner() : number_of_sampling_points_(150), is_solver_initialized_(false)
{
  solver_ptr_.reset(new qpOASES::QProblemB(number_of_sampling_points_ * 2));
  solver_ptr_->setPrintLevel(qpOASES::PL_NONE);
}

QPPlanner::~QPPlanner()
{
}

double QPPlanner::calculate2DDistace(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2)
{
  double dx = point1(0) - point2(0);
  double dy = point1(1) - point2(1);
  double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return distance;
}

void QPPlanner::doPlan(const geometry_msgs::TransformStamped& lidar2map_tf,
                       const geometry_msgs::PoseStamped& in_current_pose, const grid_map::GridMap& grid_map,
                       const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints_in_lidar,
                       const std::vector<geometry_msgs::Point>& subscribed_points_in_lidar,
                       std::vector<autoware_msgs::Waypoint>& out_waypoints,
                       std::vector<geometry_msgs::Point>& debug_interpolated_points,
                       std::vector<geometry_msgs::Point>& debug_interpolated_points2,
                       std::vector<geometry_msgs::Point>& debug_interpolated_points3)
{
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin2 = std::chrono::high_resolution_clock::now();

  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (const auto& point : in_reference_waypoints_in_lidar)
  {
    tmp_x.push_back(point.pose.pose.position.x);
    tmp_y.push_back(point.pose.pose.position.y);
  }
  ReferencePath reference_path(tmp_x, tmp_y, 0.2);
  // std::cerr << "reference path size " << reference_path.x_.size() << std::endl;

  for (size_t i = 0; i < number_of_sampling_points_; i++)
  {
    geometry_msgs::Point point;
    point.x = reference_path.x_[i];
    point.y = reference_path.y_[i];
    debug_interpolated_points.push_back(point);
  }

  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout << "frenet transform " << elapsed_time2.count() / (1000.0 * 1000.0) << " milli sec" << std::endl;

  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];

  size_t min_index = 0;
  double min_dist = 99999;
  for (size_t i = 0; i < 30; i++)
  {
    double dx = subscribed_points_in_lidar[i].x - reference_path.x_.front();
    double dy = subscribed_points_in_lidar[i].y - reference_path.y_.front();
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      min_index = i;
    }
  }
  double first_yaw = 0;
  if ((min_index + 1) < subscribed_points_in_lidar.size())
  {
    double dx = subscribed_points_in_lidar[min_index + 1].x - subscribed_points_in_lidar[min_index].x;
    double dy = subscribed_points_in_lidar[min_index + 1].y - subscribed_points_in_lidar[min_index].y;
    first_yaw = std::atan2(dy, dx);
  }
  else
  {
    std::cerr << "oh my god" << std::endl;
  }

  min_index = 0;
  min_dist = 99999;
  // for (size_t i = 0; i < subscribed_points_in_lidar.size(); i++)
  for (size_t i = 0; i < 30; i++)
  {
    double dx = subscribed_points_in_lidar[i].x - reference_path.x_.back();
    double dy = subscribed_points_in_lidar[i].y - reference_path.y_.back();
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      min_index = i;
    }
  }
  double last_yaw = 0;
  if ((min_index + 1) < subscribed_points_in_lidar.size())
  {
    double dx = subscribed_points_in_lidar[min_index + 1].x - subscribed_points_in_lidar[min_index].x;
    double dy = subscribed_points_in_lidar[min_index + 1].y - subscribed_points_in_lidar[min_index].y;
    last_yaw = std::atan2(dy, dx);
  }
  else
  {
    std::cerr << "oh my god" << std::endl;
  }
  
  for (int i = 0; i < number_of_sampling_points_ * 2; ++i)
  {
    if (i == 0)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if (i == 1)
    {
      // lower_bound[i] = reference_path.x_[i];
      // upper_bound[i] = reference_path.x_[i];
      lower_bound[i] = reference_path.x_[i - 1] + 0.2 * std::cos(first_yaw);
      upper_bound[i] = reference_path.x_[i - 1] + 0.2 * std::cos(first_yaw);
    }
    else if (i == number_of_sampling_points_ - 2)
    {
      // lower_bound[i] = reference_path.x_[i];
      // upper_bound[i] = reference_path.x_[i];
      lower_bound[i] = reference_path.x_[i + 1] - 0.2 * std::cos(last_yaw);
      upper_bound[i] = reference_path.x_[i + 1] - 0.2 * std::cos(last_yaw);
    }
    else if (i == number_of_sampling_points_ - 1)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if (i == number_of_sampling_points_)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else if (i == number_of_sampling_points_ + 1)
    {
      // lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      // upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_ - 1] + 0.2 * std::sin(first_yaw);
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_ - 1] + 0.2 * std::sin(first_yaw);
    }
    else if (i == number_of_sampling_points_ * 2 - 2)
    {
      // lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      // upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_ + 1] - 0.2 * std::sin(last_yaw);
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_ + 1] - 0.2 * std::sin(last_yaw);
    }
    else if (i == number_of_sampling_points_ * 2 - 1)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else
    {
      if (i < number_of_sampling_points_)
      {
        lower_bound[i] = reference_path.x_[i] - 0.5;
        upper_bound[i] = reference_path.x_[i] + 0.5;
      }
      else
      {
        lower_bound[i] = reference_path.y_[i - number_of_sampling_points_] - 0.5;
        upper_bound[i] = reference_path.y_[i - number_of_sampling_points_] + 0.5;
      }
    }
  }
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  std::cout << "e-osqp solve " << elapsed_time4.count() / (1000.0 * 1000.0) << " milli sec" << std::endl;

  printf("Status:                %s\n", (&workspace)->info->status);
  printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  for (size_t i = 0; i < number_of_sampling_points_; i++)
  {
    geometry_msgs::Pose pose_in_lidar_tf;
    pose_in_lidar_tf.position.x = workspace.solution->x[i];
    pose_in_lidar_tf.position.y = workspace.solution->x[i + number_of_sampling_points_];
    pose_in_lidar_tf.position.z = in_reference_waypoints_in_lidar.front().pose.pose.position.z;
    pose_in_lidar_tf.orientation.w = 1.0;
    geometry_msgs::Pose pose_in_map_tf;
    tf2::doTransform(pose_in_lidar_tf, pose_in_map_tf, lidar2map_tf);
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.pose = pose_in_map_tf;
    if (pose_in_lidar_tf.position.x > 0)
    {
      out_waypoints.push_back(waypoint);
    }
  }
}