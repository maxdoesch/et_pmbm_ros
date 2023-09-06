#include <ros/ros.h>
#include "TrackerNode.h"

int main(int argc, char** argv)
{
    ROS_INFO("ET-PMBM");

    ros::init(argc, argv, "et_pmbm_ros");

    TrackerNode tracker_node;
    tracker_node.run();
    
}