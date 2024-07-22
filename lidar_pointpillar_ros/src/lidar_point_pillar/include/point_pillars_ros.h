/**
 * @file: point_pillars_ros.h
 * @brief: ROS interface for PointPillars
 * @author: Peter
 * @date: 2023.06.26
*/


#ifndef POINT_PILLARS_ROS_H
#define POINT_PILLARS_ROS_H
#include<memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include"point_pillars.h"

class PointPillarsROS
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_objects_;
    // tf

    // PointPillar
    std::string modelFile_;
    cudaStream_t stream_ = 0;

    // methods
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void pubObjects(std::vector<Bndbox>& pred_boxs,const std_msgs::Header& header);

    std::unique_ptr<PointPillar> point_pillar_ptr_;



public:
    PointPillarsROS();
    void createROSPubSub();


};







#endif // POINT_PILLARS_ROS_H