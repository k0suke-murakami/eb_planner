/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef eb_planner_H
#define eb_planner_H


namespace autoware_msgs
{
  ROS_DECLARE_MESSAGE(Waypoint); 
}

namespace geometry_msgs
{
  ROS_DECLARE_MESSAGE(TransformStamped); 
}

namespace grid_map
{
  class GridMap;
}

namespace qpOASES
{
  class QProblemB;
}


class QPPlanner
{
public:

  QPPlanner();
  ~QPPlanner();
  
  void doPlan(const geometry_msgs::TransformStamped& lidar2map_tf,
              const geometry_msgs::PoseStamped& in_current_pose,
              const grid_map::GridMap& grid_map,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              const std::vector<geometry_msgs::Point>& subscribed_points_in_map,
              std::vector<autoware_msgs::Waypoint>& out_waypoints,
              std::vector<geometry_msgs::Point>& debug_interpolated_points,
              std::vector<geometry_msgs::Point>& debug_interpolated_points2,
              std::vector<geometry_msgs::Point>& debug_interpolated_points3);
  
  
  
private:
  int number_of_sampling_points_;
  std::unique_ptr<std::vector<geometry_msgs::Point>> previous_points_in_lidar_ptr_;

  bool prepareQP();

  double calculate2DDistace(const Eigen::Vector2d& point1,
                            const Eigen::Vector2d& point2);
};

#endif
