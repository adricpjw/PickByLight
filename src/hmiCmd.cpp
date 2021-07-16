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

void hmiCtrl::initROSFunctions() {
  lightcmd_pub_ = nh_.advertise<std_msgs::Int32>(lightcmd_topic_, 10);
  timer_ =
      nh_.createTimer(ros::Duration(1 / timer_rate_), &hmiCtrl::timerCB, this);
  objDetected_sub_ =
      nh_.subscribe(objDetected_sub_topic_, 1, &hmiCtrl::objDetectedCB, this);
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
}

void hmiCtrl::setupBoundaries() {
  transformStamped_ = tfBuffer_.lookupTransform(
      origin_frame_, lightarray_frame_, ros::Time::now(), ros::Duration(1));
  lower_x_ = transformStamped_.transform.translation.x - 0.25;
  ROS_INFO("X BOUNDARY LOWER : %f", lower_x_);
  upper_x_ = transformStamped_.transform.translation.x + 0.25;
  ROS_INFO("X BOUNDARY UPPER : %f", upper_x_);
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
  allFrames_.clear();
  filteredFrames_.clear();
  activeObjects_.clear();
  ignoredObjects_ = 0;
}
void hmiCtrl::getFrames() {
  tfBuffer_._getFrameStrings(allFrames_);
  ROS_INFO("Number of frames : %d", allFrames_.size());
  for (auto frame : allFrames_) {
    filteredFrames_.emplace_back(frame, 1);
  }
}

void hmiCtrl::filterbyPrefix(vpsi &frames) {
  for (auto &frame : frames) {
    if (frame.second == 0)
      continue;
    if (frame.first.compare(0, 3, "obj") != 0)
      frame.second = 0;
  }
}

void hmiCtrl::filterbyY(vecWaste &frames) {
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
    if (fabs(transformStamped_.transform.translation.y - y_boundary_) >
        y_tolerance_) {
      ignoredObjects_ |= (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8)));
    }
  }
}

void hmiCtrl::filterbyX(vecWaste &frames) {
  for (auto &frame : frames) {
    if (ignoredObjects_ & (1 << (frame.obj_num % (__SIZEOF_LONG_LONG__ * 8))))
      continue;
    try {
      transformStamped_ =
          tfBuffer_.lookupTransform(origin_frame_, frame.obj_id, ros::Time(0));
    } catch (tf2::ExtrapolationException &e) {
      // ignoredObjects_ &= ~(1 << (frame.obj_num  % (__SIZEOF_LONG_LONG__ *
      // 8)));
      continue;
    } catch (tf2::LookupException &e) {
      // ignoredObjects_ &= ~(1 << (frame.obj_num  % (__SIZEOF_LONG_LONG__ *
      // 8)));
      continue;
    }
    double x = transformStamped_.transform.translation.x;
    double width = frame.boundingBox[2];
    int lower_idx = (x - width/2) < lower_x_ ? 0 : ((x - width/2 - lower_x_) / diode_coverage_);
    int upper_idx = (x + width/2) < lower_x_ ? 0 : ((x + width/2 - lower_x_) / diode_coverage_);
    // int idx = x < lower_x_ ? 0 : ((x - lower_x_) / diode_coverage_);
    
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
  ROS_INFO("Object detected");
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  // filterbyPrefix(filteredFrames_);
  std::chrono::steady_clock::time_point mid = std::chrono::steady_clock::now();
  // filterbyY(filteredFrames_);
  filterbyY(activeObjects_);
  std::chrono::steady_clock::time_point mid2 = std::chrono::steady_clock::now();
  // filterbyX(filteredFrames_);
  // filterbyType(activeObjects_);
  filterbyX(activeObjects_);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  int time_diff_mid =
      std::chrono::duration_cast<std::chrono::microseconds>(mid - begin)
          .count();
  int time_diff_mid2 =
      std::chrono::duration_cast<std::chrono::microseconds>(mid2 - mid).count();
  int time_diff_end =
      std::chrono::duration_cast<std::chrono::microseconds>(end - mid2).count();
  ROS_INFO("Time elapsed : %d, %d, %d", time_diff_mid, time_diff_mid2,
           time_diff_end);
}

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

void hmiCtrl::timerCB(const ros::TimerEvent &) {
  // std::chrono::steady_clock::time_point begin =
  // std::chrono::steady_clock::now(); ROS_INFO("Refreshing containers...");
  // reset();
  // ROS_INFO("Requesting Objects...");
  // getFrames();
  // std::chrono::steady_clock::time_point mid =
  // std::chrono::steady_clock::now(); ROS_INFO("Filtering Objects...");
  // filterObj();
  // ROS_INFO("Sending command...");
  // if (prevcmd_ != cmd_)
  //   sendCmd();
  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); int time_diff_mid =
  // std::chrono::duration_cast<std::chrono::microseconds>(mid - begin).count();
  // int time_diff_end =
  // std::chrono::duration_cast<std::chrono::microseconds>(end - mid).count();
  // ROS_INFO("Time elapsed to mid : %d, Time elapsed to end : %d",
  // time_diff_mid,time_diff_end);
}

void hmiCtrl::objDetectedCB(const obj_tf::WasteItemArr::ConstPtr &msg) {

  reset();
  activeObjects_ = msg->objects;
  filterObj();
  if (prevcmd_ != cmd_)
    sendCmd();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hmiCmd");
  hmiCtrl node;
  ros::spin();
  return 0;
}
