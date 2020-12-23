#pragma once

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/package.h>
#include <ros/topic.h>

#include <sensor_msgs/CameraInfo.h>

#include "industrial_msgs/RobotStatus.h"


class StatusMonitor
{
public:
    StatusMonitor(ros::NodeHandle* n);
    // ~StatusMonitor();
    
    int checkNodeStatus();
private:
    int setParam(std::vector<std::string>& nodes, std::vector<bool>& isAction);
    int readNodeList(const std::string &path);
    void timerCB(const ros::TimerEvent& event);

    void readTxt();

    void setTopicList();

private:
    ros::Subscriber left_camera_monitor_sub_;
    ros::Subscriber right_camera_monitor_sub_;
    ros::Subscriber left_robot_state;
    ros::Subscriber right_robot_state;

private:
    void rosInit();
    void leftCameraMonitorSubCB(const sensor_msgs::CameraInfoConstPtr& msg);
    void rightCameraMonitorSubCB(const sensor_msgs::CameraInfoConstPtr& msg);
    void leftRobotState(const industrial_msgs::RobotStatusConstPtr& msg);
    void rightRobotState(const industrial_msgs::RobotStatusConstPtr& msg);

private:
    ros::NodeHandle* nh_;
    ros::Timer timer;

    std::vector<std::string> node_list_;
    std::vector<bool> node_action_list_;

    std::vector<std::string> topic_list_;
    std::vector<int> topic_action_cnt_list_;
    std::vector<bool> topic_action_list_;
};


