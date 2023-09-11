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
        void publishVisualization(std::vector<validation::ValidationModel*> const& models);

        int const _max_hypotheses = 10;

        tracker::PMBM _tracker;

        ros::Subscriber _pointSub;
        ros::Publisher _visPub;

        std::string const _pointTopic = "/rt40/dynamic_segmented";
        std::string const _visTopic = "/rt40/bbox";
};