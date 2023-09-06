#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "tracker/Tracker.h"
#include "validation/Visualization.h"

class TrackerNode : ros::NodeHandle
{
    public:
        TrackerNode();
        void run() {ros::spin();}

    private:
        void _rcvPointCallback(const sensor_msgs::PointCloud2::ConstPtr & pointMsg);

        tracker::PMBM _tracker;

        ros::Subscriber _pointSub;
        ros::Publisher _boxPub;

        ros::Time _lastIteration;

        int const _max_hypotheses = 10;

        std::string const _pointTopic = "/rt40/floor_segmented";
};