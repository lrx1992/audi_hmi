#ifndef WEB_UTILS_
#define WEB_UTILS_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "utils/point.h"
using namespace std;

namespace Poco {
namespace Net {
class HTTPServerRequest;
}
}  // namespace Poco

typedef struct
{
  float qx;
  float qy;
  float qz;
  float qw;
}QUATERNION_Type;

struct PointOfCloud{
  Point point;
  float intensity;
  double timestamp;
  uint16_t ring;
  PointOfCloud(const unsigned char* pdata) {
    point = Point(
      *((const float*)pdata),
      *((const float*)(pdata+4)),
      *((const float*)(pdata+8))
    );
    intensity = *((const float *)(pdata+16));
   // timestamp = *((const double*)(pdata+24));
    ring = *((const uint16_t *)(pdata+20));
  }
};

namespace AdUtil {
std::string GetDefaultPageContent();
std::vector<std::string> split(const std::string& str, const std::string& pattern);
std::string RequestWSAddress(const Poco::Net::HTTPServerRequest& request);
std::string RequestWebPage(const std::string& page_path);
void QuaternionToRpy(QUATERNION_Type *q, float *roll, float *pitch, float *yaw);
void calculate_pos(std::string, std::string, double, float&, float&, float&);
int send_by_udp(std::string ip, uint16_t port, std::string content);
}  

#endif
