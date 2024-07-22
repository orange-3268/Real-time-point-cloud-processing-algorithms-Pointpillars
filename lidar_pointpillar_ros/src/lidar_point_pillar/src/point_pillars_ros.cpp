#include<memory>
#include<cmath>
#include<cstdio>
#include<vector>
#include<iostream>
#include<algorithm>
#include <tf/transform_datatypes.h>
#include"autoware_msgs/DetectedObjectArray.h"

#include"point_pillars_ros.h"

PointPillarsROS::PointPillarsROS()
{
    // 初始化PointPillars
    nh_.param<std::string>("model_file",modelFile_,"/home/autoware/src/lidar_point_pillar/model/pointpillar.onnx");
    point_pillar_ptr_ = std::make_unique<PointPillar>(modelFile_,stream_);
}

void PointPillarsROS::createROSPubSub()
{
    sub_points_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_raw",1,&PointPillarsROS::pointsCallback,this);
    pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects",1);
}

void PointPillarsROS::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    // TODO：
    // 1. 推理
    // PointCloud2->pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input,*cloud);
    std::vector<Bndbox> nms_pred;
    point_pillar_ptr_->detect(cloud,nms_pred);
    // 2. 发布
    pubObjects(nms_pred,input->header);
}

void PointPillarsROS::pubObjects(std::vector<Bndbox>& pred_boxs,const std_msgs::Header& header)
{
    autoware_msgs::DetectedObjectArray objects;
    objects.header = header;
    int num_objects = pred_boxs.size();
    for(int i = 0 ; i < num_objects; i++){
        autoware_msgs::DetectedObject object;
        if(pred_boxs[i].id == 0){
            object.label = "Car";
            object.header = header;
            object.valid = true;
            object.pose_reliable = true;
            object.score = pred_boxs[i].score;
            object.pose.position.x = pred_boxs[i].x;
            object.pose.position.y = pred_boxs[i].y;
            object.pose.position.z = pred_boxs[i].z;

            float yaw = pred_boxs[i].rt;
            yaw+=M_PI/2;
            yaw = std::atan2(std::sin(yaw),std::cos(yaw));
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
            object.pose.orientation = q;

            object.dimensions.x = pred_boxs[i].l;
            object.dimensions.y = pred_boxs[i].w;
            object.dimensions.z = pred_boxs[i].h;
            objects.objects.push_back(object);
        }
    }

    pub_objects_.publish(objects);
}