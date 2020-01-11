#include "utils/msg_callback.h"
#include "utils/web_utils.h"

map<int,string> decision = {{0,"cur_lane"},{-1,"left_lane"},\
            {1,"right_lane"}};
extern map<string,AdUtil::ThreadSafeQueue<nlohmann::json>> send_queues;
extern map<string,string> send_queues_flag;
int drive_mode = 0,  lamp_status = 0, drive_mode_his = -255; 
string kombi_ip = "192.168.1.136", pi_ip = "192.168.1.66", pi_ip2 = "192.168.1.65";

void ctsCallback(const v2x::Cars_Status::ConstPtr msg){
  autodrive_msgs::CarStatus car_status;
  nlohmann::json status_json;
  for(int i = 0; i< 3; i++)
  {
    if(i==0)
    {
      car_status = msg->self_status;
    }
    else if(i==1)
    {
      if(msg->vehicle1_id == "") car_status = msg->self_status;
      else car_status = msg->vehicle1_status;
    }
    else if(i==2)
    {
      if(msg->vehicle2_id == "") car_status = msg->self_status;
      else car_status = msg->vehicle2_status;
    }
    float roll,pitch,yaw;
    QUATERNION_Type quat = {car_status.orientation.x,car_status.orientation.y,\
                            car_status.orientation.z,car_status.orientation.w};
    AdUtil::QuaternionToRpy(&quat,&roll,&pitch,&yaw);
    double time_sec = car_status.header.stamp.toSec();
    status_json["api"] = "/info_car";
    status_json["cars"][i]["id"] = i;  
    status_json["cars"][i]["velocity"]["x"] = car_status.velocity.x;
    status_json["cars"][i]["velocity"]["y"] = car_status.velocity.y;
    status_json["cars"][i]["velocity"]["z"] = car_status.velocity.z;
    status_json["cars"][i]["position"]["x"] = car_status.position.x;
    status_json["cars"][i]["position"]["y"] = car_status.position.y;
    status_json["cars"][i]["position"]["z"] = car_status.position.z;
    status_json["cars"][i]["orientation"]["r"] = roll;
    status_json["cars"][i]["orientation"]["p"] = pitch;
    status_json["cars"][i]["orientation"]["y"] = yaw;

    status_json["cars"][i]["steer_angle"] = car_status.steer_angle;
    status_json["cars"][i]["left_trun_light"] = int(car_status.left_light_status);
    status_json["cars"][i]["right_turn_light"] = int(car_status.right_light_status);
    status_json["cars"][i]["brake_light_status"] = int(car_status.brake_light_status); 
    if(i==0)
    {
      status_json["cars"][i]["car_flag"] = 2;
      if(msg->self_leader_flag == 1) status_json["cars"][i]["car_flag"] = 3;
    }
    else
    {
      status_json["cars"][i]["car_flag"] = 0;
      if(i==1 && msg->vehicle1_leader_flag==1) status_json["cars"][i]["car_flag"] = 1;
      if(i==2 && msg->vehicle2_leader_flag==1) status_json["cars"][i]["car_flag"] = 1;
    }
    
    status_json["cars"][i]["join_flag"] = 0;
    status_json["cars"][i]["join_follow_flag"] = 0;
    status_json["cars"][i]["dodge_flag"] = 0;
  }
  send_queues["car_status"].Clear();
  send_queues["car_status"].Push(status_json);
  send_queues_flag["car_status"] = "new";
}

// void ctsCallback(const autodrive_msgs::CarStatus::ConstPtr msg){
//   autodrive_msgs::CarStatus car_status = *msg;
//   nlohmann::json status_json;
//   for(int i =0; i< 3; i++)
//   {
//     float roll,pitch,yaw;
//     QUATERNION_Type quat = {car_status.orientation.x,car_status.orientation.y,\
//                             car_status.orientation.z,car_status.orientation.w};
//     AdUtil::QuaternionToRpy(&quat,&roll,&pitch,&yaw);
//     double time_sec = car_status.header.stamp.toSec();
    
//     status_json["api"] = "/info_car";
//     status_json["cars"][i]["id"] = i;  
//     status_json["cars"][i]["velocity"]["x"] = car_status.velocity.x;
//     status_json["cars"][i]["velocity"]["y"] = car_status.velocity.y;
//     status_json["cars"][i]["velocity"]["z"] = car_status.velocity.z;
//     status_json["cars"][i]["position"]["x"] = car_status.position.x;
//     status_json["cars"][i]["position"]["y"] = car_status.position.y;
//     status_json["cars"][i]["position"]["z"] = car_status.position.z;
//     status_json["cars"][i]["orientation"]["r"] = roll;
//     status_json["cars"][i]["orientation"]["p"] = pitch;
//     status_json["cars"][i]["orientation"]["y"] = yaw;
//     status_json["cars"][i]["steer_angle"] = car_status.steer_angle;
//     status_json["cars"][i]["left_trun_light"] = int(car_status.left_light_status);
//     status_json["cars"][i]["right_turn_light"] = int(car_status.right_light_status);
//     status_json["cars"][i]["brake_light_status"] = int(car_status.brake_light_status); 
//     if(i==0)
//     {
//       status_json["cars"][i]["car_flag"] = 3;
//     }
//     else
//     {
//       status_json["cars"][i]["car_flag"] = 0;
//     }
    
//     status_json["cars"][i]["join_flag"] = 0;
//     status_json["cars"][i]["join_follow_flag"] = 0;
//     status_json["cars"][i]["dodge_flag"] = 0;
//   }
//   send_queues["car_status"].Clear();
//   send_queues["car_status"].Push(status_json);
//   send_queues_flag["car_status"] = "new";
// }

void planCallback(const autodrive_msgs::PlanningTraj::ConstPtr msg){
  nlohmann::json plan_traj;
  plan_traj["api"] = "/info_planning";
  plan_traj["plan"]["decision"] = decision[msg->decision];
  int count=0;
  for(auto &point : msg->trajectory)
  {
    plan_traj["plan"]["trajectory"][count]["x"] = point.x;
    plan_traj["plan"]["trajectory"][count]["y"] = point.y;
    plan_traj["plan"]["trajectory"][count]["z"] = point.z;
    count++;
  }
  send_queues["plan_traj"].Clear();
  send_queues["plan_traj"].Push(plan_traj);
  send_queues_flag["plan_traj"] = "new";
}

void obsCallback(const autodrive_msgs::Obstacles::ConstPtr msg){
  nlohmann::json obs_json;
  obs_json["api"] = "/info_obstacle";
  int obs_count = 0;
  for(auto &obs : msg->obs)
  {
    obs_json["data"][obs_count]["type"]="car";
    obs_json["data"][obs_count]["id"] = obs.ObsId;
    obs_json["data"][obs_count]["position"]["x"]=round(obs.ObsPosition.x*100)/100.0;
    obs_json["data"][obs_count]["position"]["y"]=round(obs.ObsPosition.y*100)/100.0;
    obs_json["data"][obs_count]["position"]["z"]=round(obs.ObsPosition.z*100)/100.0;
    obs_json["data"][obs_count]["orientation"]["r"]=0;
    obs_json["data"][obs_count]["orientation"]["p"]=0;
    obs_json["data"][obs_count]["orientation"]["y"]=obs.ObsTheta;
    obs_count++;
  }
  if(obs_count == 0)
  {
    obs_json["data"] = {};
  }
  send_queues["obs_info"].Clear();
  send_queues["obs_info"].Push(obs_json);
  send_queues_flag["obs_info"]="new";
}

void mapCallback(const autodrive_msgs::LaneSection::ConstPtr msg){
  nlohmann::json map_info;
  map_info["api"] = "/info_map";
  map_info["speed_limit"] = 120;
  int min_land_id = 20, max_land_id = -20, lane_id_top=50;
  int min_land_index = 0, max_land_index = 0;  
  int lane_count=0;
  for(auto &lane : msg->lanes)
  {
    map_info["lane"][lane_count]["line_tag"] = "T";
    map_info["lane"][lane_count]["line_id"] = lane.globalid.laneId;
    map_info["lane"][lane_count]["line_type"] = lane.roadMarks[0].type;
    if(lane.globalid.laneId < min_land_id)
    {
      min_land_id = lane.globalid.laneId;
      min_land_index = lane_count;
    } 
    if(lane.globalid.laneId > max_land_id && lane.globalid.laneId < lane_id_top)
    {
      max_land_id = lane.globalid.laneId;
      max_land_index = lane_count;
    } 
    int point_count = 0;
    for(auto &point : lane.points)
    {
      map_info["lane"][lane_count]["line_point"][point_count]["x"]=point.x;
      map_info["lane"][lane_count]["line_point"][point_count]["y"]=point.y;
      map_info["lane"][lane_count]["line_point"][point_count]["z"]=point.z;
      point_count++;
    }
    lane_count++;
  }
  send_queues["map_info"].Clear();
  send_queues["map_info"].Push(map_info);
  send_queues_flag["map_info"] = "new";
  
  nlohmann::json background_info;
  background_info["api"] = "/info_background";
  int back_cnt=0;
  for(auto& i:{min_land_index,max_land_index})
  { 
    int point_count = 0;
    for(auto &point : msg->lanes[i].points)
    {
      background_info["data"][back_cnt]["line_point"][point_count]["x"]=point.x;
      if(i == min_land_index)
        background_info["data"][back_cnt]["line_point"][point_count]["y"]=point.y-10;
      else
        background_info["data"][back_cnt]["line_point"][point_count]["y"]=point.y+10;
      background_info["data"][back_cnt]["line_point"][point_count]["z"]=point.z; 
      point_count++;     
    }
    back_cnt++;
  }
  send_queues["background_info"].Clear();
  send_queues["background_info"].Push(background_info);
  send_queues_flag["background_info"] = "new";
}


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  nlohmann::json points_data;
  points_data["api"] = "/info_pointcloud";
  const int pt_step = msg->point_step;
  const int pt_cnt = msg->width;
  int display_point_cnt=0;
  for(int i = 0; i < pt_cnt; ++i)
  {
    if(i % 12 == 0)
    { 
      PointOfCloud point(&(msg->data[pt_step * i]));
      points_data["data"][display_point_cnt++] = round(point.point.x*100)/100.0;
      points_data["data"][display_point_cnt++] = round(point.point.y*100)/100.0;
      points_data["data"][display_point_cnt++] = round(point.point.z*100)/100.0;
    }    
  }
  send_queues["points_data"].Clear();
  send_queues["points_data"].Push(points_data);
  send_queues_flag["points_data"] = "new";
}

int first_speed_limit = 0;
int speed_limit = 80;
void timerCallback(const ros::TimerEvent& event,ros::Publisher& pub)
{
  if(first_speed_limit == 0){
    nlohmann::json event_data;
    event_data["api"] = "/info_event";
    event_data["data"][0]["speed_limit"] = speed_limit;
    send_queues["event_info"].Clear();
    send_queues["event_info"].Push(event_data);
    send_queues_flag["event_info"] = "new";
    first_speed_limit += 1;
  }
  autodrive_msgs::ModuleStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.status = 0;
  msg.module_name = "hmi";
  pub.publish(msg);
}

void vhcCallback(const autodrive_msgs::VehicleStatus::ConstPtr& msg)
{
  drive_mode  = msg->autodrive_mode;
 if (drive_mode  != drive_mode_his  )
 {
     if(drive_mode == 2)
     {
        AdUtil::send_by_udp(pi_ip,9090,"mode:auto");
        AdUtil::send_by_udp(pi_ip2,9090,"mode:auto");
        AdUtil::send_by_udp(pi_ip,9090,"data:1");
        AdUtil::send_by_udp(pi_ip2,9090,"data:1");
        lamp_status = 2;
     }
     else
     {
        AdUtil::send_by_udp(pi_ip,9090,"mode:manual");
        AdUtil::send_by_udp(pi_ip2,9090,"mode:manual");
         lamp_status = 8;
     }
     drive_mode_his = drive_mode;
 }
}

ModuleStatusDescription FindStatus(std::string name, int code, Parameters *parm){
  if(parm->module_map_.find(name)!=parm->module_map_.end()){
      if(parm->module_map_[name].find(code)!=parm->module_map_[name].end())
        return parm->module_map_[name].find(code)->second;
      else  return ModuleStatusDescription{"unknown status", "error"};
  }
  else  return ModuleStatusDescription{"unknown module", "error"};
}

void monitorCallback(const monitor::MonitorInfo::ConstPtr msg, Parameters *parm){
  nlohmann::json monitor_json;
  monitor_json["api"] = "/info_monitor";
  monitor_json["action"] = msg->action;
  monitor_json["cur_time"] = msg->cur_time;

  for(int i=0; i<msg->sensors.size(); i++){
    monitor_json["sensors"][i]["type"] = msg->sensors[i].type;
    monitor_json["sensors"][i]["name"] = msg->sensors[i].name;
    monitor_json["sensors"][i]["freq"] = msg->sensors[i].freq;
    monitor_json["sensors"][i]["time_diff"] = msg->sensors[i].time_diff;
    monitor_json["sensors"][i]["ip_connect_status"] = msg->sensors[i].ip_connect_status;
    monitor_json["sensors"][i]["time_out_caution"] = msg->sensors[i].time_out_caution;
  }

  for(int i=0; i<msg->modules.size(); i++){
    ModuleStatusDescription cur_modulestatus = FindStatus(msg->modules[i].name, msg->modules[i].status, parm);
    monitor_json["modules"][i]["name"] = msg->modules[i].name;
    monitor_json["modules"][i]["status_code"] = msg->modules[i].status;
    monitor_json["modules"][i]["status"] = cur_modulestatus.status;
    monitor_json["modules"][i]["freq"] = msg->modules[i].freq;
    monitor_json["modules"][i]["time_diff"] = msg->modules[i].time_diff;
    monitor_json["modules"][i]["time_out_caution"] = msg->modules[i].time_out_caution;
    monitor_json["modules"][i]["action"] = cur_modulestatus.action; 
    // cout<<"name: "<<monitor_json["modules"][i]["name"]<<
    //       " code: "<< monitor_json["modules"][i]["status_code"]<<
    //       " status: "<<monitor_json["modules"][i]["status"]<<
    //       " action: "<<monitor_json["modules"][i]["action"]<<endl;
  }

  send_queues["monitor_info"].Clear();
  send_queues["monitor_info"].Push(monitor_json);
  send_queues_flag["monitor_info"]="new";
}

void eventCallback(const autodrive_msgs::EventInfo::ConstPtr& msg)
{
  nlohmann::json event_data;
  event_data["api"] = "/info_event";
  event_data["data"][0]["content"] = msg->Reason;
  event_data["data"][0]["speed_limit"] = speed_limit;
  AdUtil::send_by_udp(kombi_ip,9999,to_string(msg->Reason));

  //event_data["data"][0]["status"] = "on" / "off";

  if(msg->Reason == 1)
  {
    lamp_status = 1;
    AdUtil::send_by_udp(pi_ip,9090,"data:0");
    AdUtil::send_by_udp(pi_ip2,9090,"data:0");
    event_data["data"][0]["status"] = "off";
  }
  else if(msg->Reason == 2)
  {
    lamp_status = 7;
    AdUtil::send_by_udp(pi_ip,9090,"data:6");
    AdUtil::send_by_udp(pi_ip2,9090,"data:6");
    event_data["data"][0]["status"] = "off";    
  }
  else if(msg->Reason == 3)
  {
    lamp_status = 2;
    AdUtil::send_by_udp(pi_ip,9090,"data:1");
    AdUtil::send_by_udp(pi_ip2,9090,"data:1");
    event_data["data"][0]["status"] = "off"; 
  }
  else if(msg->Reason == 4 or msg->Reason == 5 or msg->Reason == 6 or msg->Reason == 14)
  {
    lamp_status = 2;
    if(msg->Reason == 4)
    {
      speed_limit = 40;
      event_data["data"][0]["speed_limit"] = 40;
      AdUtil::send_by_udp(pi_ip,9090,"audio:speed_40");
    }
    if(msg->Reason == 5)
    {
      speed_limit = 60;
      event_data["data"][0]["speed_limit"] = 60;
    } 
    if(msg->Reason == 6)
    {
      speed_limit = 80;
      event_data["data"][0]["speed_limit"] = 80;
    }
    if(msg->Reason == 14)
    {
      AdUtil::send_by_udp(pi_ip,9090,"audio:pedestrian");
    }
    AdUtil::send_by_udp(pi_ip,9090,"data:1");
    AdUtil::send_by_udp(pi_ip2,9090,"data:1");
    event_data["data"][0]["status"] = "on";
  }
  else if(msg->Reason == 7)
  {
    lamp_status = 6;
    AdUtil::send_by_udp(pi_ip,9090,"data:5");
    AdUtil::send_by_udp(pi_ip2,9090,"data:5");
    AdUtil::send_by_udp(pi_ip,9090,"audio:road_close");
    event_data["data"][0]["status"] = "on";
  }
  else if(msg->Reason > 7 and msg->Reason < 14)
  {
    if(msg->LeftOrRight == -1) 
    {
      lamp_status = 3;
      AdUtil::send_by_udp(pi_ip,9090,"data:2");
      AdUtil::send_by_udp(pi_ip2,9090,"data:2");
    }
    if(msg->LeftOrRight == 1)
    {
      lamp_status = 4;
      AdUtil::send_by_udp(pi_ip,9090,"data:3");
      AdUtil::send_by_udp(pi_ip2,9090,"data:3");
    } 
    if(msg->Reason == 10)
    {
      AdUtil::send_by_udp(pi_ip,9090,"audio:road_cons");
    }
    if(msg->Reason == 12)
    {
      AdUtil::send_by_udp(pi_ip,9090,"audio:ambulance");
    }
    if(msg->Reason == 13)
    {
      AdUtil::send_by_udp(pi_ip,9090,"audio:wrong_dir");
    }   
    event_data["data"][0]["status"] = "on";    
  }
  else if(msg->Reason == 15 or msg->Reason == 16 or msg->Reason == 19)
  {
    lamp_status = 6;
    AdUtil::send_by_udp(pi_ip,9090,"data:5");
    AdUtil::send_by_udp(pi_ip2,9090,"data:5");
    event_data["data"][0]["status"] = "on";
  }
  if(msg->Reason == 17)
  {
    lamp_status = 3;
    AdUtil::send_by_udp(pi_ip,9090,"data:2");
    AdUtil::send_by_udp(pi_ip2,9090,"data:2");
    event_data["data"][0]["status"] = "off";
  }
  if(msg->Reason == 18)
  {
    lamp_status = 4;
    AdUtil::send_by_udp(pi_ip,9090,"data:3");
    AdUtil::send_by_udp(pi_ip2,9090,"data:3");
    event_data["data"][0]["status"] = "off";
  } 
  if(drive_mode != 2)   lamp_status = 8;
  send_queues["event_info"].Clear();
  send_queues["event_info"].Push(event_data);
  send_queues_flag["event_info"] = "new";
}
