#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <ClusterCenter.h>

#include <Eigen/Dense>
#include <iostream>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "AStar.hpp"

class Navigator
{
public:
    Navigator(ros::NodeHandle &nh);

    // Used to represent a 2d position
    struct Vec2d
    {
        double x, y;
        bool operator==(const Vec2d &coordinates_)
        {
            return (x == coordinates_.x && y == coordinates_.y);
        }
        bool operator!=(const Vec2d &coordinates_)
        {
            return !(*this == coordinates_);
        }
    };

    // Used to represent a point in the 2d occupancy grid
    struct Vec2i
    {
        int x, y;
        bool operator==(const Vec2i &coordinates_)
        {
            return (x == coordinates_.x && y == coordinates_.y);
        }
        bool operator!=(const Vec2i &coordinates_)
        {
            return !(*this == coordinates_);
        }
        bool operator<(const Vec2i &coordinates_) const
        {
            return x > coordinates_.x;
        }
    };

    using WayPoints = std::vector<Vec2d>;

    void uavPoseCallback(const geometry_msgs::PoseStamped &pose);
    void uavTwistCallback(const geometry_msgs::TwistStamped &twist);

    bool plan_trajectory(const WayPoints waypoints, const Eigen::VectorXd &goal_vel, mav_trajectory_generation::Trajectory *trajectory);
    bool publish_trajectory(const mav_trajectory_generation::Trajectory &trajectory);

    Vec2i world_to_grid_point(const Vec2d position, const nav_msgs::MapMetaData map_info);
    Vec2d grid_point_to_world(const Vec2i grid_point, const nav_msgs::MapMetaData map_info);
    WayPoints get_waypoints(const nav_msgs::OccupancyGridConstPtr &occupancy_msg);
    bool goal_reached();
    WayPoints get_waypoints_to_goal(const Vec2i &goal_pos, const nav_msgs::OccupancyGridConstPtr &occupancy_msg);
    std::vector<std::vector<int>> binary_matrix_from_occupancy_grid(const nav_msgs::OccupancyGridConstPtr &occupancy_msg);

    AStar::CoordinateList get_path(Vec2i current_pos, Vec2i goal_pos, const nav_msgs::OccupancyGridConstPtr &occupancy_msg);
    Vec2i sample_first_unknown_point(const nav_msgs::OccupancyGridConstPtr &occupancy_msg, const std::set<Vec2i> goalsToAvoid);
    bool valid_waypoints(const WayPoints wayPoints, const nav_msgs::OccupancyGrid::ConstPtr &msg);
    std::set<AStar::Vec2i> collision_coordinates(const AStar::Vec2i collision_grid_point, const nav_msgs::MapMetaData map_info);

    // Inline functions
    inline bool within_bounds(const Navigator::Vec2i grid_point, const nav_msgs::MapMetaData map_info)
    {
        return grid_point.x >= 0 && grid_point.y >= 0 && grid_point.x < map_info.width && grid_point.y < map_info.height;
    }

    inline bool is_unknown(const int probability)
    {
        return probability == -1;
    }

    inline bool is_occupied(const int probability)
    {
        int threshold = 50;
        return probability >= threshold;
    }

    inline bool is_free(const int probability)
    {
        return probability == 0;
    }

    inline Vec2d pose_to_2d(const Eigen::Affine3d pose)
    {
        return {current_pose.translation()[0], (double)current_pose.translation()[1]};
    }

    inline int occupancy_index(int row, int column, const nav_msgs::MapMetaData map_info)
    {
        return row + (map_info.width * column);
    }

    inline double distance(const Vec2d a, const Vec2d b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    inline Vec2d get_current_pos()
    {
        return pose_to_2d(current_pose);
    }

private:
    ros::Publisher pub_trajectory;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_twist;
    ros::NodeHandle &nh;
    Eigen::Affine3d current_pose;
    Eigen::Vector3d current_velocity;
    Vec2d goal;
};

#endif // NAVIGATION_H
