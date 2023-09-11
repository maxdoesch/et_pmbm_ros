#include "TrackerNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TrackerNode::TrackerNode() : _tracker(_max_hypotheses)
{
    _pointSub = this->subscribe(_pointTopic, 1, &TrackerNode::_rcvPointCallback, this);
    _visPub = this->advertise<visualization_msgs::MarkerArray>(_visTopic, 1);
}

void TrackerNode::_rcvPointCallback(const sensor_msgs::PointCloud2::ConstPtr & pointMsg)
{
    static ros::Time lastIteration = pointMsg->header.stamp;
    ros::Time newTimestamp = pointMsg->header.stamp;
    double time_period = (newTimestamp - lastIteration).toNSec() / 1000000000.;
    lastIteration = newTimestamp;

    std::cout << time_period << std::endl;

     _tracker.predict(time_period);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointMsg, *pointcloud);

    std::cout << pointcloud->size() << std::endl;

    if(pointcloud->size() > 0)
        _tracker.update(pointcloud);

    _tracker.reduce();

    std::vector<validation::ValidationModel*> models;
    _tracker.estimate(models);

    std::cout << models.size() << std::endl;
    publishVisualization(models);

    for(validation::ValidationModel* v_model : models)
    {
        delete v_model;
    }
}

void TrackerNode::publishVisualization(std::vector<validation::ValidationModel*> const& models)
{

    // Create marker array
    visualization_msgs::MarkerArray markers;

    // Create one marker to delete all previous markers
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(m);

    int m_id = 0; // id for the markers

    for (auto const& model : models) // iterate through all bounding boxes
    {
    
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "object_visualization";
        marker.id = m_id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = model->state()(0,0);
        marker.pose.position.y = model->state()(1,0);
        marker.pose.position.z = 0;
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(0, 0, model->getExent()[2]);
        marker.pose.orientation = tf2::toMsg(quaternion_tf2);
        marker.scale.x = model->getExent()[0];
        marker.scale.y = model->getExent()[1];
        marker.scale.z = 0.1;
        marker.color.a = 0.8;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        markers.markers.push_back(marker);
        m_id++;

        marker.id = m_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = model->state()(0,0);
        marker.pose.position.y = model->state()(1,0);
        marker.pose.position.z = 0;
        double angle =  std::atan2(model->state()[3], model->state()[2]);
        quaternion_tf2.setRPY(0, 0, angle);
        marker.pose.orientation = tf2::toMsg(quaternion_tf2);
        marker.scale.x = model->state().block<2,1>(2,0).norm();
        marker.scale.y = 0.2;
        marker.scale.z = 0.1;
        marker.color.a = 0.8;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        markers.markers.push_back(marker);

        m_id++;

    }

    _visPub.publish( markers );
}
