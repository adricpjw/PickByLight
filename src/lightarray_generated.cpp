#include "lightarray_generated.h"

LightArrBroadcaster::LightArrBroadcaster(): nh_(), private_nh_("~") {
    private_nh_.param<double>("x_pos",x,0.5);
    private_nh_.param<double>("y_pos",y,-0.1);
    private_nh_.param<double>("z_pos",z,0.3);
    private_nh_.param<double>("roll",roll,45);
    private_nh_.param<std::string>("frame_id",frame_id,"light_array");
    private_nh_.param<std::string>("origin_frame",origin_frame,"base_link");
    private_nh_.param<double>("timer_rate",timer_rate,5);
    setupTF();
    broadcastTF();
}

void LightArrBroadcaster::setupTF() {
    transformedStamped_.header.stamp = ros::Time::now();
    tf2::Quaternion quat;
    transformedStamped_.header.frame_id = origin_frame;
    transformedStamped_.child_frame_id = frame_id;
    transformedStamped_.transform.translation.x = x;
    transformedStamped_.transform.translation.y = y;
    transformedStamped_.transform.translation.z = z;
    quat.setRPY(DEG_TO_RAD*roll,0,0);
    transformedStamped_.transform.rotation.x = quat.x();
    transformedStamped_.transform.rotation.y = quat.y();
    transformedStamped_.transform.rotation.z = quat.z();
    transformedStamped_.transform.rotation.w = quat.w();

}

void LightArrBroadcaster::broadcastTF(){
    static_broadcaster.sendTransform(transformedStamped_);
    
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "lightArray");
  LightArrBroadcaster node;
  ros::spin();
  return 0;
}