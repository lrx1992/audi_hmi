/***********************************************************
*   Copyright © 2018 Horizon Robotics. All rights reserved.
*
*   Filename: parameters.h
*   Author  : penghong.lin
*   Date    : Nov 14, 2018
*   Describe: TODO
*
***********************************************************/
#ifndef SAFETY_INCLUDE_PARAM_PARAMETERS_H_
#define SAFETY_INCLUDE_PARAM_PARAMETERS_H_
#include <vector>
#include <map>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <assert.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
// using namespace std;

struct ModuleStatusDescription{
    std::string status;
    std::string action;
};
typedef std::map<unsigned int, ModuleStatusDescription> MapModuleStatus;

class Parameters {
public:
    Parameters();
    ~Parameters();
    bool initialized;
    std::string module_config_path_;
    
    // 模块状态码map
    std::map<std::string, MapModuleStatus> module_map_;
    

    void loadParams(); 
};

#endif /* SAFETY_INCLUDE_PARAM_PARAMETERS_H_ */
