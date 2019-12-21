// Copyright 2019 Horizon Robotics
// Created by xxx on 8/14/2019
// Simple web server for Audi HMI

#include <iostream>

#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/DateTimeFormat.h"
#include "Poco/Exception.h"
#include "Poco/Util/ServerApplication.h"
#include "Poco/Net/WebSocket.h"
#include "Poco/Net/NetException.h"
#include "utils/web_utils.h"
#include "utils/msg_callback.h"
#include <std_msgs/Char.h>

using Poco::Net::HTTPResponse;
using Poco::Net::HTTPRequestHandler;
using Poco::Net::HTTPRequestHandlerFactory;
using Poco::Net::HTTPServer;
using Poco::Net::HTTPServerRequest;
using Poco::Net::HTTPServerResponse;
using Poco::Util::Application;
using Poco::Net::WebSocket;
using Poco::Net::WebSocketException;
using namespace std;

extern map<string,AdUtil::ThreadSafeQueue<nlohmann::json>> send_queues;
extern map<string,string> send_queues_flag;
extern int drive_mode, lamp_status;
ros::Publisher modePub;

class HMIRequestHandler: public HTTPRequestHandler
{
public:
    HMIRequestHandler(const std::string& format): _format(format)
    {
    }
    void handleRequest(HTTPServerRequest& request, HTTPServerResponse& response)
    {
      std::string uri = request.getURI();
      if (uri == "/"){
        Application& app = Application::instance();
        app.logger().information("Request from "+ request.clientAddress().toString()+request.getURI()+ request.getVersion());
        Poco::Timestamp now;
        std::string dt(Poco::DateTimeFormatter::format(now, _format));
        response.setChunkedTransferEncoding(true);
        response.setContentType("text/json");
        std::ostream& ostr = response.send();
      }else{
        std::string content;
        content = AdUtil::RequestWebPage("");
        response.send() << content;
      }
    }
    private:
    std::string _format;
};

class WebSocketRequestHandler : public HTTPRequestHandler {
 public:
  void handleRequest(HTTPServerRequest& request, HTTPServerResponse& response) {
    Application& app = Application::instance();
    cout<<request.serverParams().getServerName()<<endl;
    try {
      WebSocket ws(request, response);
      ws.setReceiveTimeout(Poco::Timespan(0, 10000));
      ws.setSendTimeout(Poco::Timespan(5, 0));
      app.logger().information("WebSocket connection established.");
      char buffer[1024];
      int flags=0;
      string front_mode ,status_his = "";
      do{
        try {
          ws.receiveFrame(buffer, sizeof(buffer), flags);
          if(string(buffer) == "startautopilot")
          {
            std_msgs::Char mode;
            mode.data = 'g';
            modePub.publish(mode);
            continue;
          }
          else if(string(buffer) == "stopautopilot")
          {
            std_msgs::Char mode;
            mode.data = 's';    
            modePub.publish(mode);        
            continue;
          }
          if(string(buffer).substr(0,8) == "/topview") front_mode = "/topview";
          else front_mode = string(buffer);
          if(front_mode == "/driverview" || front_mode == "/topview")
          {
            for(auto &send_queue : send_queues){
              if(!send_queue.second.IsEmpty()){
                if(send_queue.first == "points_data" && front_mode == "/driverview")
                  continue;
                auto json_str = send_queue.second.WaitGetFront();
                string str = json_str->dump();
                ws.sendFrame(str.c_str(), static_cast<int>(str.size()));
                if(send_queue.first!="event_info") send_queues_flag[send_queue.first] = "old";
              }
            }
          }
        } catch (const Poco::TimeoutException& e) {
        }
        if(front_mode == "/driverview" || front_mode == "/topview"){
          for(auto &send_queue : send_queues){
            if(!send_queue.second.IsEmpty() && send_queues_flag[send_queue.first]=="new"){
              if(send_queue.first == "points_data" && front_mode == "/driverview")
                continue;              
              auto json_str_list = send_queue.second.WaitGetAll();
              for(auto &json_str : json_str_list)
              {
                string str = json_str->dump();
                ws.sendFrame(str.c_str(), static_cast<int>(str.size()));
              }
              if(send_queue.first!="event_info") send_queues_flag[send_queue.first] = "old";
            }
          }
        } 
        if(front_mode == "/status" || front_mode == "/driverview" || front_mode == "/topview")
        { 
          nlohmann::json status_json;
          status_json["api"] = "/status";
          if(drive_mode == 2)
            status_json["data"] = "autodrive";
          else
            status_json["data"] = "manual";
          status_json["lamp"] = lamp_status;
          string str = status_json.dump();
          if(status_his != str)
          {
            ws.sendFrame(str.c_str(), static_cast<int>(str.size()));
            status_his = str;
          }
        }
      } while ((flags & WebSocket::FRAME_OP_BITMASK) !=
               WebSocket::FRAME_OP_CLOSE);
      app.logger().information("WebSocket connection closed.");
    } catch (WebSocketException& exc) {
      app.logger().log(exc);
      switch (exc.code()) {
        case WebSocket::WS_ERR_HANDSHAKE_UNSUPPORTED_VERSION:
          response.set("Sec-WebSocket-Version", WebSocket::WEBSOCKET_VERSION);
          // fall through
        case WebSocket::WS_ERR_NO_HANDSHAKE:
        case WebSocket::WS_ERR_HANDSHAKE_NO_VERSION:
        case WebSocket::WS_ERR_HANDSHAKE_NO_KEY:
          response.setStatusAndReason(HTTPResponse::HTTP_BAD_REQUEST);
          response.setContentLength(0);
          response.send();
          break;
      }
    }
  }
};

class HMIRequestHandlerFactory: public HTTPRequestHandlerFactory
{
public:
    HMIRequestHandlerFactory(const std::string& format):
    _format(format)
    {
    }
    HTTPRequestHandler* createRequestHandler(const HTTPServerRequest& request)
    {

      if (request.find("Upgrade")!= request.end()) {
        // upgrade protocol
        if (Poco::icompare(request["Upgrade"], "websocket") == 0){
          return new WebSocketRequestHandler();
        }
        else{
          return nullptr;
        }
      }

      auto request_method = request.getMethod();
      if (request_method == "GET"){
        return new HMIRequestHandler(_format);
      } else
        return nullptr;
    }

private:
    std::string _format;
};

class HTTPHMIServer: public Poco::Util::ServerApplication
{
public:
    HTTPHMIServer(): _helpRequested(false)
    {
    }
    ~HTTPHMIServer()
    {
    }
protected:
    void initialize(Application& self)
        {
            loadConfiguration();
            ServerApplication::initialize(self);
        }
    void uninitialize()
        {
            ServerApplication::uninitialize();
        }
    int main(const std::vector<std::string>& args){
        if (!_helpRequested)
        {
          unsigned short port = (unsigned short)config().getInt("HTTPHMIServer.port", 8888);
          std::string format(config().getString("HTTPHMIServer.format",Poco::DateTimeFormat::SORTABLE_FORMAT));
          Poco::Net::ServerSocket svs(port);
          HTTPServer srv(new HMIRequestHandlerFactory(format),svs, new Poco::Net::HTTPServerParams);
          srv.start();
          ros::spin();
          waitForTerminationRequest();
          srv.stop();
        }
        return Application::EXIT_OK;
        }
    private:
    bool _helpRequested;
};

int main(int argc, char** argv)
{
    HTTPHMIServer app;
    ros::init(argc, argv, "web_backend",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    modePub = nh_.advertise<std_msgs::Char>("/autoDrive_KeyboardMode",1);
    ros::Subscriber vehicleSub = nh_.subscribe("/vehicle/status", 10, vhcCallback);

    ros::Subscriber carStatus = nh_.subscribe("/cars/status", 10, ctsCallback);
    //ros::Subscriber carStatus = nh_.subscribe("/car/status", 10, ctsCallback);
    AdUtil::ThreadSafeQueue<nlohmann::json> status_send_queue;
    send_queues.insert({"car_status",status_send_queue});
    send_queues_flag.insert({"car_status","old"});

    ros::Subscriber planTraj = nh_.subscribe("/planning/trajectory", 10, planCallback);
    AdUtil::ThreadSafeQueue<nlohmann::json> plan_traj_queue;
    send_queues.insert({"plan_traj",plan_traj_queue});
    send_queues_flag.insert({"plan_traj","old"});
    
    ros::Subscriber mapInfo = nh_.subscribe("/map/simple_lanes", 10, mapCallback);
    AdUtil::ThreadSafeQueue<nlohmann::json> map_info_queue;
    send_queues.insert({"map_info",map_info_queue});
    send_queues_flag.insert({"map_info","old"});

   // ros::Subscriber obsTraj = nh_.subscribe("/sensor/ibeo_front/objects",10,obsCallback);
    ros::Subscriber obsTraj = nh_.subscribe("/obstacles_in_roi_tracked_global",10,obsCallback);

    AdUtil::ThreadSafeQueue<nlohmann::json> obs_info_queue;
    send_queues.insert({"obs_info",obs_info_queue});
    send_queues_flag.insert({"obs_info","old"});

    ros::Subscriber pointCloudSub = nh_.subscribe("/sensor/velodyne_merge/points", 10, pointCloudCallback);
    AdUtil::ThreadSafeQueue<nlohmann::json> points_queue;
    send_queues.insert({"points_data",points_queue});
    send_queues_flag.insert({"points_data","old"});

    ros::Subscriber eventSub = nh_.subscribe("/command/event", 10, eventCallback);
    AdUtil::ThreadSafeQueue<nlohmann::json> event_info_queue;
    send_queues.insert({"event_info",event_info_queue});
    send_queues_flag.insert({"event_info","old"});
    
    AdUtil::ThreadSafeQueue<nlohmann::json> traffic_sign_queue;
    send_queues.insert({"traffic_sign",traffic_sign_queue});
    send_queues_flag.insert({"traffic_sign","old"});
    
    AdUtil::ThreadSafeQueue<nlohmann::json> background_queue;
    send_queues.insert({"background_info",background_queue});
    send_queues_flag.insert({"background_info","old"});

    ros::Publisher pub = nh_.advertise<autodrive_msgs::ModuleStatus>("/module_status/HMI", 1);
    ros::Timer timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&timerCallback,_1,pub));
    return app.run(argc, argv);
}
