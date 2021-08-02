#include "lineMarker.h"

LineMarker::LineMarker() : nh_(), private_nh_("~") {
    initROSFunctions();
    setupMarker();
}

void LineMarker::initROSFunctions() {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void LineMarker::setupMarker() {
    marker_.header.frame_id = "/base_link";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = "laser";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.type = visualization_msgs::Marker::LINE_LIST;
    marker_.scale.x = 0.01;
    marker_.pose.orientation.w = 1.0;
    marker_.id = 5;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.b = 0.2;
    // marker_.lifetime = ros::Duration(1);
}

void LineMarker::addPairtoMarkers(geometry_msgs::Point a, geometry_msgs::Point b) {
    marker_.points.push_back(a);
    marker_.points.push_back(b);
}

void LineMarker::publishMarkers() {
    marker_.action =visualization_msgs::Marker::ADD;
    marker_pub_.publish(marker_);
    marker_.points.clear();
}

void LineMarker::clearMarkers() {
    marker_.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_);
}


void LineMarker::clearPoints() {
    marker_.points.clear();
}