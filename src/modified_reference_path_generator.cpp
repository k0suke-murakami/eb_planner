#include <memory>
#include <autoware_msgs/Waypoint.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <distance_transform/distance_transform.hpp>
// #include <dt.h>

#include <qpOASES.hpp>
#include "osqp.h"

#include <chrono>
#include "modified_reference_path_generator.h"
#include "reference_path.h"

class Node
{
public:
  Node();
  ~Node();
  
  Eigen::Vector2d p;
  double r;
  double g;
  double h;
  double f;
  std::shared_ptr<Node> parent_node;
};

Node::Node()
{
}

Node::~Node()
{
}

struct PathPoint
{
  Eigen::Vector2d position;
  double clearance;
  double curvature;
};



//TODO: make namespace/file for utility method
//TODO: better naming 
double calculate2DDistace(const Eigen::Vector2d& point1,
                          const Eigen::Vector2d& point2)
{
  double dx = point1(0) - point2(0);
  double dy = point1(1) - point2(1);
  double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
  return distance;
}

bool compareF(Node lhs, Node rhs) { return lhs.f < rhs.f; }

std::vector<Node> expandNode(Node& parent_node, 
                             const grid_map::GridMap& clearence_map,
                             const Node& goal_node,
                             const double min_r,
                             const double max_r)
{ 
  //r min max
  double current_r;
  if(parent_node.r < min_r)
  {
    current_r = min_r;
  }
  else if(parent_node.r > max_r)
  {
    current_r = max_r;
  }
  else
  {
    current_r = parent_node.r;
  }
  
  
  std::vector<Node> child_nodes;
  Eigen::Vector2d delta_child_p;
  delta_child_p << current_r,
                   0;
  // double delta_theta = 2*M_PI/36.0;
  double delta_theta = 2*M_PI/54.0;
  for(double theta = 0; theta < 2*M_PI; theta += delta_theta)
  {
    Eigen::Matrix2d rotation;
    rotation << std::cos(theta), - std::sin(theta),
                std::sin(theta),   std::cos(theta);
    Eigen::Vector2d rotated_delta = rotation * delta_child_p;
    Node child_node;
    child_node.p = rotated_delta + parent_node.p;
    try 
    {
      double tmp_r = clearence_map.atPosition(clearence_map.getLayers().back(),
                                             child_node.p)*0.1;
      double r = std::min(tmp_r, max_r);
      if(r < min_r)
      {
        continue;
      }
      child_node.r = r;
    }
    catch (const std::out_of_range& e) 
    {
      continue;
    }
    child_node.g = parent_node.g + current_r;
    // child_node.g = parent_node.g + child_node.r;
    child_node.h = calculate2DDistace(child_node.p, goal_node.p);
    child_node.f = child_node.g + child_node.h;
    
    //hacky way to stop stacking
    if(child_node.h < child_node.f)
    {
      child_node.f = child_node.h;
    }
    
    child_node.parent_node = std::make_shared<Node>(parent_node);
    child_nodes.push_back(child_node);
  }
  return child_nodes;
}

bool isOverlap(Node node1, Node node2)
{
  double distance = calculate2DDistace(node1.p, node2.p);
  double max_r = std::max(node1.r, node2.r);
  double min_r = std::min(node1.r, node2.r);
  if((distance - max_r) < 0.5*min_r)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool nodeExistInClosedNodes(Node node, std::vector<Node> closed_nodes)
{
  for(const auto& closed_node: closed_nodes)
  {
    double distance = calculate2DDistace(node.p, closed_node.p);
    if(distance < closed_node.r)
    {
      return true;
    }
  }
  return false;
}

ModifiedReferencePathGenerator::ModifiedReferencePathGenerator(
  const double min_radius):
  min_radius_(min_radius)
{
}

ModifiedReferencePathGenerator::~ModifiedReferencePathGenerator()
{
}

double ModifiedReferencePathGenerator::calculateCurvatureFromThreePoints(
          const Eigen::Vector2d& path_point1,
          const Eigen::Vector2d& path_point2,
          const Eigen::Vector2d& path_point3)
{
  double x1 = path_point1(0);
  double y1 = path_point1(1);
  double x2 = path_point2(0);
  double y2 = path_point2(1);
  double x3 = path_point3(0);
  double y3 = path_point3(1);
  
  double a =  x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;

  double b = (x1 * x1 + y1 * y1) * (y3 - y2) 
          + (x2 * x2 + y2 * y2) * (y1 - y3)
          + (x3 * x3 + y3 * y3) * (y2 - y1);

  double c = (x1 * x1 + y1 * y1) * (x2 - x3) 
          + (x2 * x2 + y2 * y2) * (x3 - x1) 
          + (x3 * x3 + y3 * y3) * (x1 - x2);

  
  double x = -b / (2 * a);
  double y = -c / (2 * a);
  double r = std::sqrt(std::pow(x - x1, 2) + std::pow(y - y1, 2));
  
  // using cross product
  // https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product
  double plus_or_minus_sign = (x1 - x2)*(y3 - y2) - (y1 - y2)*(x3 - x2);
  Eigen::Vector2d v1, v2;
  v1 << x1 - x2, y1 - y2;
  v2 << x3 - x2, y3 - y2;
  double yaw_by_cos = std::acos(v1.dot(v2)/(v1.norm()*v2.norm()));
  
  double curvature = 0;
  //straight check
  if(yaw_by_cos > (M_PI- 0.001))
  {
    curvature = 0;
  }
  else if(plus_or_minus_sign > 0)
  {
    curvature = -1/r;
  }
  else
  {
    curvature = 1/r;
  }
  // if(1)
  // {
  //   double a = (path_point1 - path_point2).norm();
  //   double b = (path_point1 - path_point3).norm();
  //   double c = (path_point2 - path_point3).norm();
  //   double s = (a + b + c)/2;
  //   double k = std::sqrt(s*(s-a)*(s-b)*(s-c));
  //   double kpi = (4*k)/(a*b*c);
  //   // std::cerr << "curvature " << curvature << std::endl;
  //   // std::cerr << "paper " << kpi << std::endl;
  //   curvature = kpi;
  // }
  return curvature;
}

bool ModifiedReferencePathGenerator::calculateCurvatureForPathPoints(
  std::vector<PathPoint>& path_points)
{
  //calculateCurvatureFromFourPoints
  for(size_t i = 1; i < (path_points.size() - 1); i++)
  {
    double curvature = calculateCurvatureFromThreePoints(
                     path_points[i-1].position,
                     path_points[i].position,
                     path_points[i+1].position);
    path_points[i].curvature = curvature;
  }
  if(path_points.size() > 1)
  {
    path_points.front().curvature = path_points[1].curvature;
    path_points.back().curvature = path_points[path_points.size()-2].curvature;
  }
  return true;
}

double ModifiedReferencePathGenerator::calculateSmoothness(
  const std::vector<PathPoint>& path_points)
{
  double sum_smoothness = 0;
  for(size_t i = 1; i < path_points.size(); i++ )
  {
    double delta_distance = calculate2DDistace(path_points[i-1].position,
                                               path_points[i].position);
    double previous_curvature = path_points[i - 1].curvature;
    double current_curvature = path_points[i].curvature;
    double calculated_smoothness = ((std::pow(previous_curvature,2) + 
                                    std::pow(current_curvature,2))*
                                    delta_distance)/2;
    // std::cerr << "prev c " << previous_curvature << std::endl;
    // std::cerr << "curr c " << current_curvature << std::endl;
    // std::cerr << "calculated smoothness " << calculated_smoothness << std::endl;
    sum_smoothness += calculated_smoothness;
  }
  return sum_smoothness;
}

Eigen::Vector2d ModifiedReferencePathGenerator::generateNewPosition(
          const Eigen::Vector2d& parent_of_path_point1,
          const Eigen::Vector2d& path_point1,
          const Eigen::Vector2d& path_point2,
          const Eigen::Vector2d& path_point3,
          const grid_map::GridMap& clearance_map,
          const double min_r,
          const double max_k,
          const double resolustion_of_gridmap)
{
  double x1 = path_point1(0);
  double y1 = path_point1(1);
  double x2 = path_point2(0);
  double y2 = path_point2(1);
  double x3 = path_point3(0);
  double y3 = path_point3(1);
  
  const double resolution_of_gridmap = clearance_map.getResolution();
  //calculate initial e
  Eigen::Vector2d e;
  double ex, ey;
  //http://atelier-peppe.jp/programTips/GEOMETRIC/KIKA_8.html
  if(x1 == x3)
  {
    ex = x1;
    ey = y2;
  }
  else if(y1 == y3)
  {
    ex = x2;
    ey = y1;
  }
  else
  {

    double m1, m2, b1, b2;
    // 線分の傾き
    m1 = (y3 - y1) / (x3 - x1);
    // 線分のY切片
    b1 = y1 - (m1 * x1);

    // 点ptを通り、線分lineに垂直な線の傾き
    m2 = -1.0 / m1;
    // 点ptを通り、線分lineに垂直な線のY切片
    b2 = y2 - (m2 * x2);

    // 交点算出
    ex = (b2 - b1) / (m1 - m2);
    ey = (b2 * m1 - b1 * m2) / (m1 - m2);
  }
  e << ex, ey;
  
  double result_r;
  do
  {
    double k = calculateCurvatureFromThreePoints(parent_of_path_point1,
                                      path_point1,
                                      e);
    try 
    {
      double r = clearance_map.atPosition(clearance_map.getLayers().back(),
                                              e)*resolustion_of_gridmap;
      if(r > min_r && k < max_k)
      {
        return e;
      }
      else
      {
        e = (e + path_point2)/2;
      }
      
    }
    catch (const std::out_of_range& error) 
    {
      std::cerr << "e " << e << std::endl;
      std::cerr << "WARNING: could not find clearance in generateNewPostion " << std::endl;
      return path_point2;
    }
    result_r = clearance_map.atPosition(clearance_map.getLayers().back(),e)*resolution_of_gridmap;
  }while(calculate2DDistace(e, path_point2) > resolution_of_gridmap);
  return path_point2;
}

std::vector<double>  ModifiedReferencePathGenerator::generateOpenUniformKnotVector(
     const int number_of_knot,
     const int degree_of_b_spline)
{
  std::vector<double> knot_vector;
  for(size_t i = 0; i < number_of_knot; i++)
  {
    if(i < degree_of_b_spline)
    {
      knot_vector.push_back(0.0);
    }
    else if( i > number_of_knot - (degree_of_b_spline+1))
    {
      double knot = static_cast<double>(number_of_knot) - 1 - 2*static_cast<double>(degree_of_b_spline);
      knot_vector.push_back(knot);
    }
    else
    {
      double knot = static_cast<double>(i) - static_cast<double>(degree_of_b_spline);
      // std::cerr << "aaa " << knot << std::endl;
      knot_vector.push_back(knot);
    }
  }
  
  for(auto& knot: knot_vector)
  {
    knot = knot / knot_vector.back();
  }
  return knot_vector;
}


double ModifiedReferencePathGenerator::calaculateBasisFunction(
     const std::vector<double>& knot_vector,
     const size_t control_point_index,
     const size_t basis_function_index,
     const double function_value)
{
  double value = 0;
  if(basis_function_index == 0)
  {
    if(function_value > knot_vector[control_point_index] &&
       function_value <= knot_vector[control_point_index+1])
    {
      value = 1.0;
    }
    else
    {
      value = 0.0;
    }
  }
  else
  {
    double w1 = 0;
    double w2 = 0;
    if((knot_vector[control_point_index+basis_function_index+1] - 
        (knot_vector[control_point_index + 1])) != 0)
    {
      w1 = calaculateBasisFunction(knot_vector, 
                                  control_point_index+1, 
                                  basis_function_index - 1, 
                                  function_value)*
           (knot_vector[control_point_index+basis_function_index+1] - function_value)/
           (knot_vector[control_point_index+basis_function_index+1] - 
            knot_vector[control_point_index+1]); 
    }
    if((knot_vector[control_point_index+basis_function_index] - 
       knot_vector[control_point_index]) != 0)
    {
      w2 = calaculateBasisFunction(knot_vector, 
                                  control_point_index, 
                                  basis_function_index - 1, 
                                  function_value)*
           (function_value - knot_vector[control_point_index])/
           (knot_vector[control_point_index+basis_function_index] -
            knot_vector[control_point_index]); 
    }
    value = w1 + w2;
  }
  return value;
}

bool ModifiedReferencePathGenerator::generateModifiedReferencePath(
    grid_map::GridMap& clearance_map, 
    const geometry_msgs::Point& start_point, 
    const geometry_msgs::Point& goal_point,
    const geometry_msgs::TransformStamped& lidar2map_tf, 
    const geometry_msgs::TransformStamped& map2lidar_tf,
    std::vector<autoware_msgs::Waypoint>& debug_modified_smoothed_reference_path,
    std::vector<autoware_msgs::Waypoint>& debug_modified_smoothed_reference_path_in_lidar)
{
  
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin1 = std::chrono::high_resolution_clock::now();
  std::string layer_name = clearance_map.getLayers().back();
  grid_map::Matrix grid_data = clearance_map.get(layer_name);

  // grid_length y and grid_length_x respectively
  dope::Index2 size({ static_cast<unsigned>(clearance_map.getLength()[1]*10),
                      static_cast<unsigned>(clearance_map.getLength()[0]*10) });
  // dope::Index2 size({ 200, 600 });
  dope::Grid<float, 2> f(size);
  dope::Grid<dope::SizeType, 2> indices(size);
  bool is_empty_cost = true;
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j)
    {
      if (grid_data(i * size[1] + j) > 0.01)
      {
        f[i][j] = 0.0f;
        is_empty_cost = false;
      }
      else
      {
        f[i][j] = std::numeric_limits<float>::max();
      }
    }
  }

  
  // Note: this is necessary at least at the first distance transform execution
  // and every time a reset is desired; it is not, instead, when updating
  dt::DistanceTransform::initializeIndices(indices);

  
  dt::DistanceTransform::distanceTransformL2(f, f, false, 1);
  
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j)
    {
      if (is_empty_cost)
      {
        grid_data(i * size[1] + j) = 1;
      }
      else
      {
        grid_data(i * size[1] + j) = f[i][j];
      }
    }
  }
  // 3. 現在日時を再度取得
  std::chrono::high_resolution_clock::time_point end1 = std::chrono::high_resolution_clock::now();
  // 経過時間を取得
  std::chrono::nanoseconds elapsed_time1 = std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - begin1);
  std::cout <<"entire distance transform " <<elapsed_time1.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin_a_star_plus_rule_smooth = std::chrono::high_resolution_clock::now();
  
  clearance_map[layer_name] = grid_data;

  geometry_msgs::Point start_point_in_lidar_tf, goal_point_in_lidar_tf;
  tf2::doTransform(start_point, start_point_in_lidar_tf, map2lidar_tf);
  tf2::doTransform(goal_point, goal_point_in_lidar_tf, map2lidar_tf);
  
  Eigen::Vector2d start_p, goal_p;
  start_p(0) = start_point_in_lidar_tf.x; 
  start_p(1) = start_point_in_lidar_tf.y; 
  goal_p(0) = goal_point_in_lidar_tf.x; 
  goal_p(1) = goal_point_in_lidar_tf.y; 
  
  std::vector<Node> s_open;
  Node* a = new Node();
  Node initial_node;
  const double resolution_of_gridmap = clearance_map.getResolution();
  // const double min_r = 1.6;
  // const double min_r = 2.0;
  const double max_r = min_radius_+0.5;
  initial_node.p = start_p;
  double initial_r = clearance_map.atPosition(layer_name, initial_node.p) * resolution_of_gridmap;
  if(initial_r < min_radius_)
  {
    initial_r = min_radius_;
  }
  else if(initial_r > max_r)
  {
    initial_r = max_r;
  }
  initial_node.r = initial_r;
  initial_node.g = 0;
  initial_node.h = calculate2DDistace(initial_node.p, goal_p);
  initial_node.f = initial_node.g + initial_node.h;
  initial_node.parent_node = nullptr;
  s_open.push_back(initial_node);
  
  Node goal_node;
  goal_node.p = goal_p;
  double goal_r = clearance_map.atPosition(layer_name, goal_node.p) * resolution_of_gridmap;
  if(goal_r < min_radius_)
  {
    goal_r = min_radius_;
  }
  else if(goal_r > max_r)
  {
    goal_r = max_r;
  }
  goal_node.r = goal_r;
  goal_node.g = 0;
  goal_node.h = 0;
  goal_node.f = 0;
  
  double f_goal = std::numeric_limits<double>::max();
  
  std::vector<Node> s_closed;
  while(!s_open.empty())
  {
    std::sort(s_open.begin(), s_open.end(), compareF);
    Node lowest_f_node = s_open.front();
    // std::cerr << "lowest g " << lowest_f_node.g << std::endl;
    // std::cerr << "lowest h " << lowest_f_node.h << std::endl;
    // std::cerr << "lowest f " << lowest_f_node.f << std::endl;
    s_open.erase(s_open.begin());
    if(f_goal < lowest_f_node.f)
    {
      break;
    }
    else if(nodeExistInClosedNodes(lowest_f_node, s_closed))
    {
      continue;
    }
    else
    {
      std::vector<Node> child_nodes = 
          expandNode(lowest_f_node, 
                     clearance_map,
                     goal_node,
                     min_radius_,
                     max_r);
      s_open.insert(s_open.end(),
                    child_nodes.begin(),
                    child_nodes.end());
      s_closed.push_back(lowest_f_node);
      if(isOverlap(lowest_f_node, goal_node))
      {
        f_goal = lowest_f_node.f;
      }
    }
  }
  
  std::vector<PathPoint> path_points;
  
  //backtrack
  Node current_node = s_closed.back();
  double dist = calculate2DDistace(current_node.p, goal_p);
  if(dist > 5)
  {
    std::cerr << "Error: fail to find goal by graph A*; "<< dist  <<" away from goal" << std::endl;
    return false;
  }
  
  if(current_node.parent_node == nullptr)
  {
    std::cerr << "no node is explored"  << std::endl;
    return true;
  }
  
  do
  {
    PathPoint path_point;
    path_point.position = current_node.p;
    path_point.clearance = current_node.r;
    path_point.curvature = 0;
    path_points.insert(path_points.begin(), path_point);
    current_node = *current_node.parent_node;
  }while(current_node.parent_node != nullptr);
  
  
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(const auto& point: path_points)
  {
    // std::cerr << "poitn " << point.position(0) << std::endl;
    tmp_x.push_back(point.position(0));
    tmp_y.push_back(point.position(1));
    autoware_msgs::Waypoint waypoint_in_lidar;
    waypoint_in_lidar.pose.pose.position.x = point.position(0);
    waypoint_in_lidar.pose.pose.position.y = point.position(1);
    waypoint_in_lidar.pose.pose.position.z = start_point_in_lidar_tf.z;
    waypoint_in_lidar.pose.pose.orientation.z = 1.0;
    debug_modified_smoothed_reference_path_in_lidar.push_back(waypoint_in_lidar);
    
    geometry_msgs::Pose pose_in_lidar_tf;
    pose_in_lidar_tf.position.x = point.position(0);
    pose_in_lidar_tf.position.y = point.position(1);
    pose_in_lidar_tf.position.z = start_point_in_lidar_tf.z;
    pose_in_lidar_tf.orientation.w = 1.0;
    geometry_msgs::Pose pose_in_map_tf;
    tf2::doTransform(pose_in_lidar_tf, pose_in_map_tf, lidar2map_tf);
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.pose = pose_in_map_tf;
    waypoint.cost = point.curvature;
    debug_modified_smoothed_reference_path.push_back(waypoint);   
  }
  // 3. 現在日時を再度取得
  std::chrono::high_resolution_clock::time_point end_a_star_plus_rule_smooth = std::chrono::high_resolution_clock::now();
  // 経過時間を取得
  std::chrono::nanoseconds chunk_elapsed_time = 
     std::chrono::duration_cast<std::chrono::nanoseconds>(
       end_a_star_plus_rule_smooth - begin_a_star_plus_rule_smooth);
  std::cout <<"a star " <<chunk_elapsed_time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  return true;
}
