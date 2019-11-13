#include <autoware_msgs/Waypoint.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/utils.h>
// #include <tf/transform_datatypes.h>

//headers in Eigen
// #include <Eigen/Dense>
#include <distance_transform/distance_transform.hpp>
#include <grid_map_core/GridMap.hpp>
#include <qpOASES.hpp>
#include <qpOASES/QProblem.hpp>
// #include <qpoases_vendor/qpOASES.hpp>

#include <memory>

#include "reference_path.h"


#include "eb_planner.h"


QPPlanner::QPPlanner():
number_of_sampling_points_(150),
is_solver_initialized_(false)
{
  solver_ptr_.reset(new qpOASES::QProblemB(number_of_sampling_points_*2));
  solver_ptr_->setPrintLevel(qpOASES::PL_NONE);
}

QPPlanner::~QPPlanner()
{
}

double QPPlanner::calculate2DDistace(const Eigen::Vector2d& point1,
                          const Eigen::Vector2d& point2)
{
  double dx = point1(0) - point2(0);
  double dy = point1(1) - point2(1);
  double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
  return distance;
}

void QPPlanner::doPlan(
  const geometry_msgs::TransformStamped& lidar2map_tf,
  const geometry_msgs::PoseStamped& in_current_pose,
  const grid_map::GridMap& grid_map,
  const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints_in_lidar,
  std::vector<autoware_msgs::Waypoint>& out_waypoints,
  std::vector<geometry_msgs::Point>& debug_interpolated_points,
  std::vector<geometry_msgs::Point>& debug_interpolated_points2,
  std::vector<geometry_msgs::Point>& debug_interpolated_points3)
{
  // // 1. 現在日時を取得
  // std::chrono::high_resolution_clock::time_point begin1 = std::chrono::high_resolution_clock::now();

  
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin2 = std::chrono::high_resolution_clock::now();

  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(const auto& point: in_reference_waypoints_in_lidar)
  {
    tmp_x.push_back(point.pose.pose.position.x);
    tmp_y.push_back(point.pose.pose.position.y); 
  }
  ReferencePath reference_path(tmp_x, tmp_y, 0.2);
  std::cerr << "reference path size " << reference_path.x_.size() << std::endl;
  
  for (size_t i = 0; i < number_of_sampling_points_; i++)
  {
    geometry_msgs::Point point;
    point.x = reference_path.x_[i];
    point.y = reference_path.y_[i];
    debug_interpolated_points.push_back(point);
  }
  
  
  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time2 = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout <<"frenet transform " <<elapsed_time2.count()/(1000.0*1000.0)<< " milli sec" << std::endl;

  
  
  
  
  double h_matrix[number_of_sampling_points_*number_of_sampling_points_*2*2];
  double g_matrix[number_of_sampling_points_*2];
  double lower_bound[number_of_sampling_points_*2];
  double upper_bound[number_of_sampling_points_*2];
  
  int index = 0;

  for (int r = 0; r < number_of_sampling_points_*2; ++r)
  {
    for (int c = 0; c < number_of_sampling_points_*2; ++c)
    {
      if((r < number_of_sampling_points_-2 && r > 1) ||
         (r < number_of_sampling_points_*2-2 && r > number_of_sampling_points_ +1))
      {
        if(r==c)
        {
          h_matrix[index] = 6;
        }
        else if(std::abs(c - r) == 1)
        {
          h_matrix[index] = -4;
        }
        else if(std::abs(c - r) == 2)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == 1)
      {
        if(c==0)
        {
          h_matrix[index] = -2;
        }
        else if(c==1)
        {
          h_matrix[index] = 5;
        }
        else if(c==2)
        {
          h_matrix[index] = -4;
        }
        else if(c==4)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_+1)
      {
        if(c==number_of_sampling_points_*2)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_*2+1)
        {
          h_matrix[index] = 5;
        }
        else if(c==number_of_sampling_points_*2+2)
        {
          h_matrix[index] = -4;
        }
        else if(c==number_of_sampling_points_*2+3)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == 0)
      {
        if(c==0)
        {
          h_matrix[index] = 1;
        }
        else if(c==1)
        {
          h_matrix[index] = -2;
        }
        else if(c==2)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_)
      {
        if(c==number_of_sampling_points_*2)
        {
          h_matrix[index] = 1;
        }
        else if(c==number_of_sampling_points_*2+1)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_*2+2)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_-2)
      {
        if(c==number_of_sampling_points_-1)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_-2)
        {
          h_matrix[index] = 5;
        }
        else if(c==number_of_sampling_points_-3)
        {
          h_matrix[index] = -4;
        }
        else if(c==number_of_sampling_points_-4)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_*2-2)
      {
        if(c==number_of_sampling_points_*2-1)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_*2-2)
        {
          h_matrix[index] = 5;
        }
        else if(c==number_of_sampling_points_*2-3)
        {
          h_matrix[index] = -4;
        }
        else if(c==number_of_sampling_points_*2-4)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_-1)
      {
        if(c==number_of_sampling_points_-1)
        {
          h_matrix[index] = 1;
        }
        else if(c==number_of_sampling_points_-2)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_-3)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else if(r == number_of_sampling_points_*2-1)
      {
        if(c==number_of_sampling_points_*2-1)
        {
          h_matrix[index] = 1;
        }
        else if(c==number_of_sampling_points_*2-2)
        {
          h_matrix[index] = -2;
        }
        else if(c==number_of_sampling_points_*2-3)
        {
          h_matrix[index] = 1;
        }
        else
        {
          h_matrix[index] = 0;
        }
      }
      else
      {
        h_matrix[index] = 0;
      }
      index++;
    }
  }
  
  Eigen::MatrixXd tmp_b = Eigen::MatrixXd::Identity(number_of_sampling_points_, number_of_sampling_points_);
  for (int i = 0; i < number_of_sampling_points_*2; ++i)
  {
    
    if(i == 0)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if(i == 1)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if(i == number_of_sampling_points_-2)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if(i == number_of_sampling_points_-1)
    {
      lower_bound[i] = reference_path.x_[i];
      upper_bound[i] = reference_path.x_[i];
    }
    else if(i == number_of_sampling_points_)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else if(i == number_of_sampling_points_+1)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else if(i == number_of_sampling_points_*2-2)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else if(i == number_of_sampling_points_*2-1)
    {
      lower_bound[i] = reference_path.y_[i - number_of_sampling_points_];
      upper_bound[i] = reference_path.y_[i - number_of_sampling_points_];
    }
    else
    {
      lower_bound[i] = -1000;
      upper_bound[i] =  1000;
    }
    g_matrix[i] = tmp_b(i);
  }
  
  
  int max_iter = 500;
  
  std::chrono::high_resolution_clock::time_point begin3 = std::chrono::high_resolution_clock::now();
  if(!is_solver_initialized_)
  {
    auto ret = solver_ptr_->init(h_matrix, g_matrix, 
                          lower_bound, upper_bound, 
                          max_iter); 
    is_solver_initialized_ = true;
  }
  else
  {
    std::cerr << "warm start"  << std::endl;
    auto ret = solver_ptr_->hotstart(g_matrix, 
                                     lower_bound, upper_bound, max_iter);
  }
  
  
  double result[number_of_sampling_points_];
  solver_ptr_->getPrimalSolution(result);
  
  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end3 = std::chrono::high_resolution_clock::now();  
  std::chrono::nanoseconds elapsed_time3 = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout <<"solve " <<elapsed_time3.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  // write here later
  std::cerr << "dddd"  << std::endl;
  for(size_t i = 0; i < number_of_sampling_points_; i++)
  {
    
    geometry_msgs::Pose pose_in_lidar_tf;
    pose_in_lidar_tf.position.x = result[i];
    pose_in_lidar_tf.position.y = result[i+number_of_sampling_points_];
    std::cerr << "resutl y " << result[i] << std::endl;
    pose_in_lidar_tf.position.z = in_reference_waypoints_in_lidar.front().pose.pose.position.z;
    pose_in_lidar_tf.orientation.w = 1.0;
    geometry_msgs::Pose pose_in_map_tf;
    tf2::doTransform(pose_in_lidar_tf, pose_in_map_tf, lidar2map_tf);
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.pose = pose_in_map_tf;
    out_waypoints.push_back(waypoint); 
  }
}