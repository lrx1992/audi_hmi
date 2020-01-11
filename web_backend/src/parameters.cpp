#include <utils/parameters.h>

using namespace std;
std::vector<std::string> split(const std::string& s, const std::string& delim) {
    std::vector<std::string> v;
    if (!delim.empty()) {
        auto b = s.begin();
        auto i = s.end();
        while ((i = std::search(b, s.end(), delim.begin(), delim.end())) != s.end()) {
            if (i - b > 0) {
                v.emplace_back(std::string(b, i));
            }
            b = i + delim.length();
        }
        if (b != s.end()) {
            v.emplace_back(std::string(b, s.end()));
        }
    } else {
        throw "Delimiter can't be empty string";
    }
    return v;
}

Parameters::Parameters() {
    char *ros_package_path;
    ros_package_path = getenv("ROS_PACKAGE_PATH");
    std::string package_paths = ros_package_path;
    std::string delimiter = ":";
    std::vector<std::string> paths = split(package_paths, delimiter);
    if(paths.size() < 2) {
        throw "Please source setup.bash under catkin workspace!";
        return;
    }
    module_config_path_ = paths[0] + "/safety/monitor/parameters/module_status_par.yaml";
    loadParams();
}

Parameters::~Parameters() {
}

void Parameters::loadParams(){
    //加载传感器配置文件

    //加载各模块状态列表
    YAML::Node module_node = YAML::LoadFile(module_config_path_);
    assert(!module_node.IsNull());

    assert(module_node["MODULE_ENABLE"]);
    YAML::Node module_enable = module_node["MODULE_ENABLE"];

    /*************** 构建map **************/
    for(int i=0; i<module_enable.size(); i++){
        assert(module_enable[i]["module"]);
        string module_name = module_enable[i]["module"].as<std::string>();
        const YAML::Node& module_yaml_info = module_node[module_name];
        MapModuleStatus cur_module_map;
        for (int i = 0; i< module_yaml_info.size(); i++){
            ModuleStatusDescription module_tmp;
            assert(module_yaml_info[i]["status"]); 
            module_tmp.status = module_yaml_info[i]["status"].as<std::string>();
            assert(module_yaml_info[i]["action"]); 
            module_tmp.action = module_yaml_info[i]["action"].as<std::string>();
            assert(module_yaml_info[i]["code"]); 
            unsigned int status_code = module_yaml_info[i]["code"].as<unsigned int>();
            cur_module_map[status_code] = module_tmp;
        }
        module_map_[module_name] = cur_module_map;
    }
    cout<<"_map_: "<<module_map_.size()<<endl;
    cout<<"_can_: "<<module_map_["can"].size()<<endl;
    // cout<<"_can_: "<<module_map_["can"][255].status<<endl;
}

