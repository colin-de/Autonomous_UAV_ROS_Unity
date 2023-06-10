#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"

#include "ros/ros.h"

class cloud2voxel
{
public:
  ros::NodeHandle nh;
  cloud2voxel() : nh("~")
  {
    subscriber_points_ = nh.subscribe("/points/raw", 5, &cloud2voxel::point_cb, this);
    publisher_points_ = nh.advertise<sensor_msgs::PointCloud2>("/points/filtered", 1);
    ros::spin();
  }
  ~cloud2voxel(){};

private:
  ros::Subscriber subscriber_points_;
  ros::Publisher publisher_points_;

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);

    // Rotating the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    double rotx = 0.0;
    double roty = M_PI / 2.0;
    double rotz = -M_PI / 2.0;
    Eigen::Matrix4f rotMatrixX;
    Eigen::Matrix4f rotMatrixY;
    Eigen::Matrix4f rotMatrixZ;

    rotMatrixX << 1.0, 0.0, 0.0, 0.0,
        0.0, cos(rotx), -sin(rotx), 0.0,
        0.0, sin(rotx), cos(rotx), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrixY << cos(roty), 0.0, sin(roty), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin(roty), 0.0, cos(roty), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrixZ << cos(rotz), -sin(rotz), 0.0, 0.0,
        sin(rotz), cos(rotz), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    sensor_msgs::PointCloud2 pub_pc;
    pcl::transformPointCloud(*filtered_pc_ptr, *transformed_cloud_ptr, rotMatrixX * rotMatrixY * rotMatrixZ);
    pcl::toROSMsg(*transformed_cloud_ptr, pub_pc);

    pub_pc.header = in_cloud_ptr->header;

    publisher_points_.publish(pub_pc);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cloud_voxel");
  cloud2voxel node;
}
