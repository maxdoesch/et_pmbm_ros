#include "TrackerNode.h"

#include <pcl_conversions/pcl_conversions.h>

TrackerNode::TrackerNode() : _tracker(_max_hypotheses), _visualization(0.001, 40), _lastIteration{ros::Time::now()}
{
    _pointSub = this->subscribe(_pointTopic, 1, &TrackerNode::_rcvPointCallback, this);
}

void TrackerNode::_rcvPointCallback(const sensor_msgs::PointCloud2::ConstPtr & pointMsg)
{
    double time_period_ms = (ros::Time::now() - _lastIteration).toNSec() / 1000000.;
    _lastIteration = ros::Time::now();

    _tracker.predict(time_period_ms);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointMsg, *pointcloud);

    if(pointcloud->size() > 0)
        _tracker.update(pointcloud);

    _tracker.reduce();

    std::vector<validation::ValidationModel*> models;
    _tracker.estimate(models);

    _visualization.draw(pointcloud, models);

    for(validation::ValidationModel* v_model : models)
    {
        delete v_model;
    }
}