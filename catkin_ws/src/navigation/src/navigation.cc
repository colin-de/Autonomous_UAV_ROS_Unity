#include <Navigation.h>

Navigator::Navigator(ros::NodeHandle &nh)
    : nh(nh),
      current_velocity(Eigen::Vector3d::Zero()),
      current_pose(Eigen::Affine3d::Identity()),
      goal({-1, -1})
{

  pub_trajectory = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("/trajectory", 0);
  sub_pose = nh.subscribe("/true_pose", 1, &Navigator::uavPoseCallback, this);
  sub_twist = nh.subscribe("/true_twist", 1, &Navigator::uavTwistCallback, this);
}

// Callback to get current Pose of UAV
void Navigator::uavPoseCallback(const geometry_msgs::PoseStamped &pose)
{
  tf::poseMsgToEigen(pose.pose, current_pose);
}

void Navigator::uavTwistCallback(const geometry_msgs::TwistStamped &twist)
{
  // store current velocity
  tf::vectorMsgToEigen(twist.twist.linear, current_velocity);
}

bool Navigator::publish_trajectory(const mav_trajectory_generation::Trajectory &trajectory)
{
  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
  msg.header.frame_id = "world";
  pub_trajectory.publish(msg);

  return true;
}

Navigator::Vec2i Navigator::world_to_grid_point(const Navigator::Vec2d position, const nav_msgs::MapMetaData map_info)
{
  auto grid_x = (unsigned int)((position.x - map_info.origin.position.x) / map_info.resolution);
  auto grid_y = (unsigned int)((position.y - map_info.origin.position.y) / map_info.resolution);
  return {grid_x, grid_y};
}

Navigator::Vec2d Navigator::grid_point_to_world(const Navigator::Vec2i grid_point, const nav_msgs::MapMetaData map_info)
{
  auto x = map_info.resolution * grid_point.x + map_info.origin.position.x;
  auto y = map_info.resolution * grid_point.y + map_info.origin.position.y;
  return {x, y};
}

bool Navigator::plan_trajectory(const WayPoints waypoints, const Eigen::VectorXd &goal_vel, mav_trajectory_generation::Trajectory *trajectory)
{
  const int dimension = 3;
  const double max_velocity = 0.2;
  const double max_acceleration = 0.1;
  const int z_value = 5;
  Eigen::VectorXd start_pos(dimension);
  Eigen::VectorXd goal_pos(dimension);
  start_pos << waypoints[0].x, waypoints[0].y, z_value;
  goal_pos << waypoints[waypoints.size() - 1].x, waypoints[waypoints.size() - 1].y, z_value;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  start.makeStartOrEnd(start_pos, derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity);

  // add waypoint to list
  vertices.push_back(start);

  Eigen::VectorXd pos_desired(dimension);
  for (int i = 1; i < waypoints.size() - 1; i++)
  {
    pos_desired << waypoints[i].x, waypoints[i].y, z_value;
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos_desired);
    vertices.push_back(middle);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
  }

  end.makeStartOrEnd(goal_pos, derivative_to_optimize);

  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);

  // add waypoint to list
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_velocity, max_acceleration);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_velocity);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_acceleration);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));

  return true;
}

// Find the first unknown location in the occupancy grid
Navigator::Vec2i Navigator::sample_first_unknown_point(const nav_msgs::OccupancyGridConstPtr &occupancy_msg, const std::set<Vec2i> goalsToAvoid)
{
  nav_msgs::MapMetaData map_info = occupancy_msg->info;
  auto occupancy_grid = occupancy_msg->data;
  for (int row = 0; row < map_info.width; row++)
  {
    for (int column = 0; column < map_info.height; column++)
    {
      int current_cell_value = occupancy_grid[occupancy_index(row, column, map_info)];
      if (is_unknown(current_cell_value))
      {
        Navigator::Vec2i possible_goal = {row, column};
        if (goalsToAvoid.count(possible_goal) == 0)
        {
          std::cout << "possible goal:" << possible_goal.x << " " << possible_goal.y << "\n";
          return possible_goal;
        }
      }
    }
  }
  return world_to_grid_point(goal, map_info);
}

// Finds the next goal location, and return a set of waypoints from the current location to this goal if a collision-free path is found
Navigator::WayPoints Navigator::get_waypoints(const nav_msgs::OccupancyGridConstPtr &occupancy_msg)
{
  nav_msgs::MapMetaData map_info = occupancy_msg->info;
  auto binaryMatrix = binary_matrix_from_occupancy_grid(occupancy_msg);
  ClusterCenter cluster_center(binaryMatrix, map_info.height, map_info.width);
  auto location_of_most_unknowns = cluster_center.location_of_most_unknowns();
  Vec2i possible_goal = {location_of_most_unknowns.one, location_of_most_unknowns.two};

  WayPoints waypoints = get_waypoints_to_goal(possible_goal, occupancy_msg);
  std::set<Vec2i> goals_to_avoid = {};
  int attempts = 0;
  while (waypoints.size() < 2 && attempts < 1000)
  {
    goals_to_avoid.insert(possible_goal);
    possible_goal = sample_first_unknown_point(occupancy_msg, goals_to_avoid);
    waypoints = get_waypoints_to_goal(possible_goal, occupancy_msg);
    attempts++;
  }
  if (waypoints.size() > 0)
  {
    goal = waypoints[waypoints.size() - 1];
  }
  return waypoints;
}

bool Navigator::goal_reached()
{
  auto current_pos = pose_to_2d(current_pose);
  double threshold = 0.5;
  return distance(current_pos, goal) < threshold;
}

// Gets the path generated by A* and converts it into the world frame
Navigator::WayPoints Navigator::get_waypoints_to_goal(const Vec2i &goal_pos, const nav_msgs::OccupancyGridConstPtr &occupancy_msg)
{
  auto map_info = occupancy_msg->info;
  auto current_pos = world_to_grid_point(pose_to_2d(current_pose), map_info);
  std::cout << "Current position in grid coordinates: " << current_pos.x << ", " << current_pos.y << std::endl;
  std::cout << "Possible goal in grid coordinates: " << goal_pos.x << ", " << goal_pos.y << std::endl;

  AStar::CoordinateList path = get_path(current_pos, goal_pos, occupancy_msg);
  WayPoints waypoints;

  std::cout << "Path in grid coordinates: " << std::endl;
  for (auto coordinate : path)
  {
    std::cout << coordinate.x << " " << coordinate.y << "\n";
    waypoints.push_back(grid_point_to_world({coordinate.x, coordinate.y}, map_info));
  }

  return waypoints;
}

// Converts the occupancy grid to a binary matrix, which has the value of 1 if the cell is unknown, 0 otherwise
std::vector<std::vector<int>> Navigator::binary_matrix_from_occupancy_grid(const nav_msgs::OccupancyGridConstPtr &occupancy_msg)
{
  nav_msgs::MapMetaData map_info = occupancy_msg->info;
  auto occupancy_grid = occupancy_msg->data;
  std::vector<std::vector<int>> matrix;
  int num_rows = map_info.width;
  int num_cols = map_info.height;
  matrix.resize(num_rows, std::vector<int>(num_cols, 0));
  for (int row = 0; row < num_rows; row++)
  {
    for (int column = 0; column < num_cols; column++)
    {
      int current_cell_value = occupancy_grid[occupancy_index(row, column, map_info)];
      if (is_unknown(current_cell_value))
      {
        matrix[row][column] = 1;
      }
    }
  }
  return matrix;
}

// Uses A* to generate a path from current position to goal position. The occupied cells in the occupancy grid are set as obstacles
AStar::CoordinateList Navigator::get_path(Vec2i current_pos, Vec2i goal_pos, const nav_msgs::OccupancyGridConstPtr &occupancy_msg)
{
  auto occupancy_grid = occupancy_msg->data;
  auto map_info = occupancy_msg->info;
  int rows = map_info.width;
  int columns = map_info.height;
  AStar::Generator generator;
  generator.setWorldSize({rows, columns});
  generator.setHeuristic(AStar::Heuristic::euclidean);
  generator.setDiagonalMovement(true);

  for (int row = 0; row < rows; row++)
  {
    for (int column = 0; column < columns; column++)
    {
      int current_cell_value = occupancy_grid[row + (map_info.width * column)];
      if (!is_free(current_cell_value))
      {
        for (auto &coordinate : collision_coordinates({row, column}, map_info))
        {
          if (!(coordinate.x == current_pos.x && coordinate.y == current_pos.y))
          {
            generator.addCollision(coordinate);
          }
        }
      }
    }
  }

  return generator.findPath({goal_pos.x, goal_pos.y}, {current_pos.x, current_pos.y});
}

// Returns the coordinates around a collision point, since the drone might collide even when its position is not directly at an occupied cell
std::set<AStar::Vec2i> Navigator::collision_coordinates(const AStar::Vec2i collision_grid_point, const nav_msgs::MapMetaData map_info)
{
  std::set<AStar::Vec2i> coordinates;
  auto drone_width = 0.83;
  auto collision_box_width = drone_width / map_info.resolution * 2;

  for (int x = collision_grid_point.x - collision_box_width / 2; x < collision_grid_point.x + collision_box_width / 2; x++)
  {
    for (int y = collision_grid_point.y - collision_box_width / 2; y < collision_grid_point.y + collision_box_width / 2; y++)
    {
      if (within_bounds({x, y}, map_info))
      {
        coordinates.insert({x, y});
      }
    }
  }

  return coordinates;
}

// Check if the waypoints are free of collisions with the updated map
bool Navigator::valid_waypoints(const WayPoints wayPoints, const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  for (Vec2d waypoint : wayPoints)
  {
    Vec2i gridPoint = world_to_grid_point(waypoint, msg->info);
    int probability = msg->data[occupancy_index(gridPoint.x, gridPoint.y, msg->info)];
    if (is_occupied(probability))
    {
      return false;
    }
  }
  return true;
}