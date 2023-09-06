#pragma once

#include <ros/ros.h>

#include "tracker/Tracker.h"

class TrackerNode : ros::NodeHandle
{
    public:
        TrackerNode();
        void run() {ros::spin();}

    private:
        tracker::PMBM _tracker;

        ros::Subscriber _pointSub;
        

        int const _max_hypotheses = 10;
};