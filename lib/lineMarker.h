#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>



class LineMarker {

public:
    LineMarker();
    void publishMarkers();
    void clearMarkers();
    void addPairtoMarkers(geometry_msgs::Point a, geometry_msgs::Point b);
    void clearPoints();
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber lightcmd_sub_;
    visualization_msgs::Marker marker_;

    /*-----------------_PARAMS-----------------*/
    std::string lightcmd_sub_topic_;
    std::string origin_frame_;

    void initROSFunctions();

    void setupMarker();

    
};