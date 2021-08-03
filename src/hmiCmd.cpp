#include "hmiCmd.h"
#include <chrono>

hmiCtrl::hmiCtrl()
    : nh_(), private_nh_("~"), tfBuffer_(ros::Duration(5)),
      tflistener_(tfBuffer_), marker_(), activeObjects_({}),
      ignoredObjects_(0) {
  initParams();
  initROSFunctions();
  setupBoundaries();
  setupLights();
}

/*----------- INITIALIZATION -----------*/

void hmiCtrl::initROSFunctions() {
  lightcmd_pub_ = nh_.advertise<std_msgs::Int32>(lightcmd_topic_, 10);
  objDetected_sub_ = nh_.subscribe(objDetected_sub_topic_, 1, &hmiCtrl::objDetectedCB, this);
}

void hmiCtrl::initParams() {
  
  private_nh_.param<std::string>("lightcmd_topic", lightcmd_topic_,
                                 "/lightcmd");
  private_nh_.param<std::string>("lightarray_frame", lightarray_frame_,
                                 "light_array");
  private_nh_.param<std::string>("origin_frame", origin_frame_, "/lightcmd");
  private_nh_.param<std::string>("objDetected_sub_topic",
                                 objDetected_sub_topic_, "/activeObjects");
  private_nh_.param<double>("num_lights", num_lights_, 3);
  private_nh_.param<double>("timer_rate", timer_rate_, 5);
  private_nh_.param<double>("y_tolerance", y_tolerance_, 0.05);
  private_nh_.param<double>("array_width", array_width_, 0.44);
  private_nh_.param<bool>("FIL_BY_X", XFIL_EN_, true);
  private_nh_.param<bool>("FIL_BY_Y", YFIL_EN_, true);
  private_nh_.param<bool>("FIL_BY_TYPE", TYPEFIL_EN_, true);
}

void hmiCtrl::setupBoundaries() {
  /* -----------------SETTING UP COORDINATE BOUNDARIES ----------------------*/
  transformStamped_ = tfBuffer_.lookupTransform(
      origin_frame_, lightarray_frame_, ros::Time::now(), ros::Duration(1));

  /* --- X AXIS BOUNDARIES ---*/
  lower_x_ = transformStamped_.transform.translation.x - array_width_ / 2;
  ROS_INFO("X BOUNDARY LOWER : %f", lower_x_);
  upper_x_ = transformStamped_.transform.translation.x + array_width_ / 2;
  ROS_INFO("X BOUNDARY UPPER : %f", upper_x_);

  /* --- Y AXIS BOUNDARIES ---*/
  // Y-AXIS at which the lights are pointing on,
  // calculated from angle of descension and height of array
  double yaw, pitch, roll;
  tf2::getEulerYPR(transformStamped_.transform.rotation, yaw, pitch, roll);
  frame_height_ = transformStamped_.transform.translation.z;
  frame_y_ = transformStamped_.transform.translation.y;
  y_boundary_ = frame_y_ + (frame_height_ / tan(-roll));
  ROS_INFO("Y BOUNDARY : %f", y_boundary_);
}

void hmiCtrl::setupLights() { diode_coverage_ = belt_width_ / num_lights_; }

void hmiCtrl::reset() {
  prevcmd_ = cmd_;
  cmd_ = 0;
  activeObjects_.clear();
  ignoredObjects_ = 0;
}



/*----------- FILTERS ---------*/


void hmiCtrl::filterbyY(vecWaste &frames) {
  // std::cout<< "outer"<<std::endl;
  for (auto &frame : frames) {
    if (ignoredObjects_ & (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8))))
      continue;
    try {
      transformStamped_ =
          tfBuffer_.lookupTransform(origin_frame_, frame.obj_id, ros::Time(0));
    } catch (tf2::ExtrapolationException &e) {
      continue;
    } catch (tf2::LookupException &e) {
      continue;
    }
    // std::cout<<"inner" <<std::endl;
    double height = frame.boundingBox[3];
    double yPose = transformStamped_.transform.translation.y;
    // std::cout<< frame.obj_id << " >> yPose : " << yPose<<std::endl;
    // std::cout<<"Difference : " <<fabs(yPose - y_boundary_) <<std::endl;
    if (fabs(yPose - y_boundary_) >
        (y_tolerance_ + height / 2)) {
          // std::cout <<"ignoring"<<std::endl;
      ignoredObjects_ |= (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8)));
    }
  }
}

void hmiCtrl::filterbyX(vecWaste &frames) {
  
  for (auto &frame : frames) {
    std::cout<<ignoredObjects_<<std::endl;
    if (ignoredObjects_ & (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8))))
      continue;
    // std::cout<<"reaching x filter"<<std::endl;

    try {
      transformStamped_ =
          tfBuffer_.lookupTransform(origin_frame_, frame.obj_id, ros::Time(0));
    } catch (tf2::ExtrapolationException &e) {
      continue;
    } catch (tf2::LookupException &e) {
      continue;
    }
    // std::cout<<"x filter"<<std::endl;
    double x = transformStamped_.transform.translation.x;
    double width = frame.boundingBox[2];
    int lower_idx = (x - width / 2) < lower_x_
                        ? 0
                        : ((x - width / 2 - lower_x_) / diode_coverage_);
    int upper_idx = (x + width / 2) < lower_x_
                        ? 0
                        : ((x + width / 2 - lower_x_) / diode_coverage_);
    std::cout << "x : " << x << ", width = : " << width << std::endl;
    for (int i = lower_idx; i <= upper_idx; i++) {
      cmd_ |= (1 << i);
      addVisualisation(i);
    }
  }
}

void hmiCtrl::filterbyType(vecWaste &frames) {
  for (auto &frame : frames) {
    if (ignoredObjects_ & (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8))))
      continue;
    if (frame.plastictype == 1)
      ignoredObjects_ |= (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8)));
  }
}

void hmiCtrl::filterObj() {
  // ROS_INFO("Object detected");
  /*------ SETTING UP FILTERS ---- */

  if (YFIL_EN_)
    filterbyY(activeObjects_);
  if (TYPEFIL_EN_)
    filterbyType(activeObjects_);
  if (XFIL_EN_)
    filterbyX(activeObjects_);
}

/*----------- PUBLISHERS -----------*/

void hmiCtrl::sendCmd() {
  lightcmd_.data = cmd_;
  lightcmd_pub_.publish(lightcmd_);
  marker_.clearMarkers();
  if (cmd_)
    marker_.publishMarkers();
  marker_.clearPoints();
}

void hmiCtrl::addVisualisation(int idx) {
  geometry_msgs::Point point1, point2;
  point1.x = point2.x =
      lower_x_ + (idx * diode_coverage_ + (diode_coverage_ / 2));
  point1.z = frame_height_;
  point2.z = 0.0;
  point1.y = frame_y_;
  point2.y = y_boundary_;
  marker_.addPairtoMarkers(point1, point2);
}

/*----------- SUBSCRIBER ---------*/

void hmiCtrl::objDetectedCB(const obj_tf::WasteItemArr::ConstPtr &msg) {
  reset();
  activeObjects_ = msg->objects;
  filterObj();
  if (prevcmd_ != cmd_)
    sendCmd();
}

/*----------- MAIN ---------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "hmiCmd");
  hmiCtrl node;
  ros::spin();
  return 0;
}
