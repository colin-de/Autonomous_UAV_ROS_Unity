#include <Navigation.h>

class navigationNode
{
public:
    ros::NodeHandle nh;
    navigationNode() : nh("~")
    {
        initialized = false;
        navigator = new Navigator(nh);
        goal_velocity << 0.0, 0.0, 0.0;
        vis_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 5);
        map_sub = nh.subscribe("/octomap_2d/projected_map", 10, &navigationNode::mapCallback, this);

        int timeout_treshold = 200;
        ros::Rate loop_rate(0.1);
        // Main loop. Starts by generating waypoints once the occupancy grid has been received
        // Continously checks if new waypoints should be generated, which is the case when
        // the drone has reached its goal, or when it has been stationary for too long.
        // Waypoints are also regenerated when the new map shows that the previous path is not collision-free anymore
        // The waypoints are set as positional constraints in the trajectory generation
        while (ros::ok())
        {
            if (occupancy_message)
            {
                if (!initialized)
                {
                    ROS_INFO("Navigator initialized");
                    initialized = true;
                    generate_waypoints();
                    reset_timer();
                }
                else if (navigator->goal_reached())
                {
                    ROS_INFO("Goal reached");
                    generate_waypoints();
                    reset_timer();
                }
                else if (!navigator->valid_waypoints(waypoints, occupancy_message))
                {
                    ROS_INFO("Invalid waypoints");
                    generate_waypoints();
                    reset_timer();
                }
                else if (count >= timeout_treshold)
                {
                    ROS_INFO("Timeout");
                    if (drone_stationary())
                    {
                        ROS_INFO("Drone stationary");
                        generate_waypoints();
                    }
                    reset_timer();
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
    }
    ~navigationNode(){};

private:
    ros::Publisher vis_pub;
    ros::Subscriber map_sub;
    Eigen::Vector3d goal_velocity;
    Navigator *navigator;
    nav_msgs::OccupancyGrid::ConstPtr occupancy_message;
    bool initialized;
    Navigator::Vec2d last_drone_position;
    int count;
    Navigator::WayPoints waypoints;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        occupancy_message = msg;
    }

    void reset_timer()
    {
        last_drone_position = navigator->get_current_pos();
        count = 0;
    }

    bool drone_stationary()
    {
        double threshold = 0.5;
        return navigator->distance(last_drone_position, navigator->get_current_pos()) < threshold;
    }

    void generate_waypoints()
    {
        ROS_INFO("Generating new wayPoints");
        waypoints = navigator->get_waypoints(occupancy_message);
        if (waypoints.size() >= 2)
        {
            mav_trajectory_generation::Trajectory trajectory;
            navigator->plan_trajectory(waypoints, goal_velocity, &trajectory);
            navigator->publish_trajectory(trajectory);
        }
        visualize_waypoints();
    }

    void visualize_waypoints()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Waypoints";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        geometry_msgs::Point pt;
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        for (int i = 0; i < waypoints.size(); i++)
        {
            pt.x = waypoints[i].x;
            pt.y = waypoints[i].y;
            pt.z = 5;
            marker.points.push_back(pt);

            if (i == 0)
            {
                c.r = 1.0;
            }
            else if (i == waypoints.size() - 1)
            {
                c.r = 0;
                c.b = 1.0;
            }
            else
            {
                c.g = 1.0;
            }
            marker.colors.push_back(c);
        }
        vis_pub.publish(marker);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");

    navigationNode node;
}