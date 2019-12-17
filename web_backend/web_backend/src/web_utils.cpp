#include "utils/web_utils.h"
#include <Poco/Net/HTTPServerRequest.h>
#include <fstream>
#include <utils/json.h>
#include "utils/threadsafe_queue.h"
#include <tuple>
#define ODO_PI 3.1415926
map<string,AdUtil::ThreadSafeQueue<nlohmann::json>> send_queues;
map<string,string> send_queues_flag;

namespace AdUtil{

std::string GetDefaultPageContent() {
  return std::string{
      "<html>"
      "<head>"
      "<title>Audi HMI - Web</title>"
      "</head>"
      "<body>"
      "  <h1>Audi HMI - Web</h1>"
      "  <p>Cannot Find \"web_res/index.html\"</p>"
      "</body>"
      "</html>"};
}

std::string RequestWSAddress(const Poco::Net::HTTPServerRequest& request) {
  auto address = request.serverAddress().toString();
  nlohmann::json j;
  j["ws"] = address;
  return j.dump();
}

std::string RequestWebPage(const std::string& page_path) {
  std::ifstream ifs(page_path);
  if (ifs.is_open()) {
    return std::string(std::istreambuf_iterator<char>(ifs),
                       std::istreambuf_iterator<char>());
  } else {
    std::cout << "Can't find resource " << page_path << ", use default.\n";
    return AdUtil::GetDefaultPageContent();
  }
}

void QuaternionToRpy(QUATERNION_Type *q, float *roll, float *pitch, float *yaw)
{
  /* roll (x-axis rotation) */
  float sinr = +2.0 * (q->qw * q->qx + q->qy * q->qz);
  float cosr = +1.0 - 2.0 * (q->qx * q->qx + q->qy * q->qy);
  *roll = atan2f(sinr, cosr);
  /* pitch (y-axis rotation) */
  float sinp = +2.0 * (q->qw * q->qy - q->qz * q->qx);
  if (fabs(sinp) > 1.0)
  {
    /* use 90 degrees if out of range */
    *pitch = copysign(ODO_PI / 2.0, sinp);
  }
  else
  {
    *pitch = asinf(sinp);
  }
  /* yaw (z-axis rotation) */
  float siny = +2.0 * (q->qw * q->qz + q->qx * q->qy);
  float cosy = +1.0 - 2.0 * (q->qy * q->qy + q->qz * q->qz);
  *yaw = atan2f(siny, cosy);
}
void calculate_pos(std::string cnt_flag, std::string move_flag, double now_time, float& pos_x, float& pos_y, float& pos_yaw)
{
  static double follow_vel = 23;
  static double last_time;
  static float last_x,last_y;
  static std::vector<vector<float>> join_points={{200.774, 48.415},{206.026,49.692},{219.162,52.664},
  {229.875,54.710},{245.101,58.215},{249.255,59.066},{261.028,61.033},{271.535,62.410},{280.907,63.469},
  {291.395,65.036},{306.986,66.695},{320.159,67.405},{333.100,68.228},{340.263,68.647},{351.358,69.104},
  {362.916,69.451},{372.623,69.895},{382.333,70.108},{393.543,70.567},{402.324,71.117},{408.330,71.755},
  {416.185,72.412},{420.335,73.609},{426.342,74.132}};
  static std::vector<vector<float>> dodge_points={{1021.991, 74.545},{1027.304, 73.458},{1034.549, 72.371},
  {1043.122, 71.647},{1050.367, 71.526},{1064.254, 71.043},{1073.552, 71.043},{1080.676, 70.681}, 
  {1089.128, 70.802},{1098.184, 70.922},{1107.482, 70.560},{1115.935, 70.198},{1125.112, 69.956}, 
  {1133.685, 69.594},{1142.138, 69.473},{1149.987, 68.990},{1157.835, 68.749},{1166.651, 68.387}, 
  {1176.311, 67.904},{1183.314, 67.662},{1189.956, 66.817},{1201.668, 65.972},{1213.744, 64.885}, 
  {1225.819, 63.194},{1235.720, 61.745},{1247.192, 60.055},{1255.282, 58.727},{1263.614, 56.915}};
  std::vector<vector<float>> cal_set;
  if(move_flag == "join") cal_set = join_points;
  else cal_set = dodge_points; 
  int set_len = cal_set.size();
  if(cnt_flag == "first")
  {
    last_x = pos_x = cal_set[0][0];
    last_y = pos_y = cal_set[0][1];
    last_time = now_time;
    pos_yaw = 0;
  }
  else
  {
    float distance = follow_vel*(now_time - last_time);
    for(int i =0; i< (set_len-1); i++)
    {
      if(last_x >= cal_set[i+1][0]) 
      {
        continue;
      }
      float old_to_next = sqrt((pow(cal_set[i+1][0]-last_x,2)+pow(cal_set[i+1][1]-last_y,2)));
      if(old_to_next > distance)
      {
        float rate = distance/old_to_next;
        pos_x = cal_set[i+1][0]*rate+(1-rate)*last_x;
        pos_y = cal_set[i+1][1]*rate+(1-rate)*last_y;
        pos_yaw = atan2f(pos_y-last_y,pos_x-last_x);
        last_x = pos_x;
        last_y = pos_y;
        last_time = now_time;
        break;
      }
      else
      {
        if(i == set_len-2)
        {
          pos_x = pos_y = pos_yaw =-1;
          break;
        }
        else
        {
          last_x = cal_set[i+1][0];
          last_y = cal_set[i+1][1];
          distance -= old_to_next;
          continue;
        }
      }
    }
  }
}
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int send_by_udp(std::string ip, uint16_t port, std::string content)
{
  int client_fd;
  struct sockaddr_in ser_addr;
  client_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(client_fd < 0)
  {
    std::cout<<"create socket fail!\n";
    return -1;
  }
  bzero(&ser_addr, sizeof(ser_addr));
  ser_addr.sin_family = AF_INET;
  ser_addr.sin_addr.s_addr = inet_addr(ip.c_str()) ;
  //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //注意网络序转换
  ser_addr.sin_port = htons(port); //注意网络序转换
  int nMsgLen = content.size();
  char* msg = (char*)malloc(nMsgLen+1);
  bzero(msg,nMsgLen+1);
  strncpy(msg,content.c_str(),nMsgLen);
  struct sockaddr* dst = (struct sockaddr*)&ser_addr;
  socklen_t len = sizeof(*dst);
  sendto(client_fd, msg, nMsgLen, 0, dst, len);
  close(client_fd);
  free(msg);
  return 0;
}
}  


