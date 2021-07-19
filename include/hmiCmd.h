#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <hmiCmd/ObjRecognised.h>
#include <hmiCmd/ObjRecognisedArr.h>
#include <obj_tf/WasteItemArr.h>
#include <queue>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <math.h>
#include "lineMarker.h"

class hmiCtrl {
typedef std::pair<std::string,int> psi;
typedef std::vector<psi> vpsi;
typedef std::vector<obj_tf::WasteItem> vecWaste;
const double belt_width_ = 0.50;
public:
  hmiCtrl();

private:
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher lightcmd_pub_;
  ros::Timer timer_;
  ros::Subscriber objDetected_sub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tflistener_;
  geometry_msgs::TransformStamped transformStamped_;

  std_msgs::Int32 lightcmd_msg_;
  std::string lightcmd_topic_;
  std::string lightarray_frame_;
  std::string origin_frame_;
  std::string objDetected_sub_topic_;
  std::vector<std::string> allFrames_;
  vecWaste activeObjects_;
  long long ignoredObjects_;
  vpsi filteredFrames_;
  double timer_rate_;
  double lower_x_;
  double upper_x_;
  double y_boundary_;
  double frame_height_;
  double frame_y_;
  double num_lights_;
  double diode_coverage_;
  double y_tolerance_;
  double array_width_;
  bool XFIL_EN_;
  bool YFIL_EN_;
  bool TYPEFIL_EN_;


  void initParams();
  void initROSFunctions();
  void setupBoundaries();
  void setupLights();

  void filterObj();
  void filterbyY(vecWaste &frames);
  void filterbyX(vecWaste &frames);
  void filterbyType(vecWaste &frames);
  void getFrames();

  void sendCmd();
  void reset();
  void timerCB(const ros::TimerEvent &);
  void objDetectedCB(const obj_tf::WasteItemArr::ConstPtr &msg);
  

  void addVisualisation(int idx);

  //Message to sent to RPI
  std_msgs::Int32 lightcmd_;
  int prevcmd_;
  int cmd_;

  // For visualization
  LineMarker marker_;
};