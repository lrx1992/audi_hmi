#ifndef MSG_CALLBACK_
#define MSG_CALLBACK_
#include "autodrive_msgs/CarStatus.h"
#include "v2x/Cars_Status.h"
#include "autodrive_msgs/PlanningTraj.h"
#include "autodrive_msgs/LaneSection.h"
#include "autodrive_msgs/Obstacles.h"
#include "autodrive_msgs/ModuleStatus.h"
#include <autodrive_msgs/VehicleStatus.h>
#include <autodrive_msgs/EventInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <monitor/MonitorInfo.h>
#include "utils/json.h"
#include "utils/threadsafe_queue.h"
#include "utils/web_utils.h"
#include <string>
#include <map>
#include "ros/ros.h"
#include <utils/parameters.h>
using namespace std;
//void ctsCallback(const autodrive_msgs::CarStatus::ConstPtr msg);
void ctsCallback(const v2x::Cars_Status::ConstPtr msg);
void planCallback(const autodrive_msgs::PlanningTraj::ConstPtr msg);
void mapCallback(const autodrive_msgs::LaneSection::ConstPtr msg);
void obsCallback(const autodrive_msgs::Obstacles::ConstPtr msg);
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void timerCallback(const ros::TimerEvent& event, ros::Publisher& pub);
void vhcCallback(const autodrive_msgs::VehicleStatus::ConstPtr& msg);
void eventCallback(const autodrive_msgs::EventInfo::ConstPtr& msg);
void monitorCallback(const monitor::MonitorInfo::ConstPtr msg, Parameters *parm);
#endif
