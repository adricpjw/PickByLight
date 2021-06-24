#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <math.h>

class LightArrBroadcaster {
    const double DEG_TO_RAD = M_PI/180;

    public:
        LightArrBroadcaster();
    private:
        ros::NodeHandle nh_, private_nh_;
        ros::Timer timer_;
        geometry_msgs::TransformStamped transformedStamped_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster;
        void broadcastTF();
        void timerCB(const ros::TimerEvent&);
        void setupTF();

        /*------------PARAMS-----------------*/
        double x,y,z;
        double roll;
        double timer_rate;
        std::string frame_id;
        std::string origin_frame;

};