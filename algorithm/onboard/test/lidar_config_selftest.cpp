#include <onboard/config_file.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdio>
#include <unistd.h>
int main(int argc,char **argv) {
  if(argc!=2)return 1;ego_onboard::AppConfig config;std::string error;
  if(!ego_onboard::loadConfigFile(argv[1],config,error)||!config.lidar.allow_zero_crc||config.lidar.expected_serial!="ARMCP8H0037373"||config.lidar.point_data_type!=1||config.lidar.frame_duration_s!=.1)return 1;
  char path[]="/tmp/uav-lidar-config-XXXXXX";int fd=mkstemp(path);if(fd<0)return 1;close(fd);
  bool ok=true;
  auto check=[&](YAML::Node root,bool expected,const std::string &reason=std::string()){
    std::ofstream out(path);out<<root;out.close();ego_onboard::AppConfig parsed;std::string err;
    const bool result=ego_onboard::loadConfigFile(path,parsed,err);
    const bool passed=result==expected&&(reason.empty()||err.find(reason)!=std::string::npos);
    if(!passed)std::printf("Config case expected=%d result=%d reason=%s error=%s\n",expected,result,reason.c_str(),err.c_str());
    ok=ok&&passed;
  };
  auto root=YAML::LoadFile(argv[1]);root["lidar"]["crc_policy"]="strict";root["lidar"]["expected_serial"]="";check(root,true);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=0.5;check(root,true);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=0;check(root,false);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=2.1;check(root,false);
  root=YAML::LoadFile(argv[1]);root["timing"]["position_extrapolation_s"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["map"]["obstacles_inflation"]=1.5;check(root,false);
  root=YAML::LoadFile(argv[1]);root["map"]["virtual_floor_height"]=".nan";check(root,false,"map.virtual_floor_height");
  root=YAML::LoadFile(argv[1]);root["map"]["virtual_floor_height"]=".inf";check(root,false,"map.virtual_floor_height");
  root=YAML::LoadFile(argv[1]);root["map"]["virtual_floor_height"]=-1;check(root,false,"map.virtual_floor_height");
  root=YAML::LoadFile(argv[1]);root["map"]["virtual_floor_height"]=11.75;check(root,false,"lower handover height");
  root=YAML::LoadFile(argv[1]);root["vehicle"]["min_obstacle_gap"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["vehicle"]["radius"]=1.4;check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["expected_serial"]="";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["crc_policy"]="anything";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["queue_capacity"]=-1;check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["frame_duration_s"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"]["min_range_m"]=100;check(root,false);
  root=YAML::LoadFile(argv[1]);root["lidar"].remove("planning_range_m");check(root,true);
  root=YAML::LoadFile(argv[1]);root["lidar"]["planning_range_m"]=".nan";check(root,false,"lidar.planning_range_m");
  root=YAML::LoadFile(argv[1]);root["lidar"]["planning_range_m"]=0;check(root,false,"lidar.planning_range_m");
  root=YAML::LoadFile(argv[1]);root["lidar"]["planning_range_m"]=101;check(root,false,"lidar.planning_range_m");
  root=YAML::LoadFile(argv[1]);root["lidar"]["max_range_m"]=30;check(root,false,"lidar.planning_range_m");
  // A large map and a 100 m cutoff must not allow a 20 m/s profile to bypass
  // the 40 m nominal planning budget, even with its old 70 m horizon restored.
  root=YAML::LoadFile(argv[1]);root["planner"]["max_vel"]=20;root["planner"]["planning_horizon"]=70;
  root["map"]["update_range_xy"]=90;root["map"]["size_xy"]=YAML::Load("[216,216]");
  check(root,false,"lidar.planning_range_m must cover");
  // Stopping run including the existing reaction budget is 24.95 m at 10 m/s.
  root=YAML::LoadFile(argv[1]);root["planner"]["max_vel"]=10;root["planner"]["planning_horizon"]=20;
  root["lidar"]["planning_range_m"]=24.94;check(root,false,"lidar.planning_range_m must cover");
  root["lidar"]["planning_range_m"]=24.96;check(root,true);
  root=YAML::LoadFile(argv[1]);root["planner"]["max_vel"]=0;check(root,false);
  root=YAML::LoadFile(argv[1]);root["planner"]["max_vel"]=".nan";check(root,false);
  root=YAML::LoadFile(argv[1]);root["planner"]["local_target_speed"]=0;check(root,false,"planner.local_target_speed");
  root=YAML::LoadFile(argv[1]);root["planner"]["local_target_speed"]=".nan";check(root,false,"planner.local_target_speed");
  root=YAML::LoadFile(argv[1]);root["planner"].remove("local_target_speed");check(root,true);
  root=YAML::LoadFile(argv[1]);root["planner"]["max_acc"]=0;check(root,false);
  root=YAML::LoadFile(argv[1]);root["planner"]["planning_horizon"]=1;check(root,false);
  root=YAML::LoadFile(argv[1]);root["map"]["update_range_xy"]=10;check(root,false,"map.update_range_xy must cover");
  root=YAML::LoadFile(argv[1]);root["map"]["size_xy"]=YAML::Load("[66,66]");check(root,false);
  unlink(path);std::printf("Measured lidar profile config and invalid settings rejected=%d\n",ok);return ok?0:1;
}
