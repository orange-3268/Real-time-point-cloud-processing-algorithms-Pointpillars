// ros节点


#include<iostream>
#include"point_pillars_ros.h"


int main(int argc,char **argv)
{
    ros::init(argc,argv,"lidar_pointpillars");
    PointPillarsROS app;
    app.createROSPubSub();
    ros::spin();
    
    return 0;
}