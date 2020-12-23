#include "status_monitor/status_monitor.h"

StatusMonitor::StatusMonitor(ros::NodeHandle *n)
{
    nh_ = n;
    readTxt();
    setTopicList();
    rosInit();
}

void StatusMonitor::timerCB(const ros::TimerEvent &event)
{
    checkNodeStatus();
    setParam(node_list_, node_action_list_);
    for (int i = 0; i < topic_action_cnt_list_.size(); i++)
    {
        --topic_action_cnt_list_[i];
        if (topic_action_cnt_list_[i] <= 0)
        {
            topic_action_list_[i] = false;
        }
        else
        {
            topic_action_list_[i] = true;
        }
    }
    setParam(topic_list_, topic_action_list_);
}

void StatusMonitor::rosInit()
{
    left_camera_monitor_sub_ = nh_->subscribe("/camera_base_left/color/camera_info", 10, &StatusMonitor::leftCameraMonitorSubCB, this);
    right_camera_monitor_sub_ = nh_->subscribe("/camera_base_right/color/camera_info", 10, &StatusMonitor::rightCameraMonitorSubCB, this);
    timer = nh_->createTimer(ros::Duration(1), &StatusMonitor::timerCB, this);
}

void StatusMonitor::setTopicList()
{
    topic_list_.push_back("/camera_base_left/color/camera_info");
    topic_list_.push_back("/camera_base_right/color/camera_info");
    topic_list_.push_back("/left_robot_connet");
    topic_list_.push_back("/left_robot_error");
    topic_list_.push_back("/left_robot_power");
    topic_list_.push_back("/right_robot_connet");
    topic_list_.push_back("/right_robot_error");
    topic_list_.push_back("/right_robot_power");
    topic_action_list_.resize(topic_list_.size());
    topic_action_cnt_list_.resize(topic_list_.size());
    for (int i = 0; i < topic_action_cnt_list_.size(); i++)
    {
        topic_action_cnt_list_[i] = 3;
    }
    nh_->setParam("/status/topic_list", topic_list_);
}

void StatusMonitor::leftCameraMonitorSubCB(const sensor_msgs::CameraInfoConstPtr &msg)
{
    topic_action_cnt_list_[0] = 3;
}

void StatusMonitor::rightCameraMonitorSubCB(const sensor_msgs::CameraInfoConstPtr &msg)
{
    topic_action_cnt_list_[1] = 3;
}

void StatusMonitor::leftRobotState(const industrial_msgs::RobotStatusConstPtr &msg)
{
    topic_action_cnt_list_[2] = 3;
    if (msg->in_error.val == 0)
    {
        topic_action_cnt_list_[3] = 3;
    }
    if (msg->drives_powered.val == 1)
    {
        topic_action_cnt_list_[4] = 3;
    }
}

void StatusMonitor::rightRobotState(const industrial_msgs::RobotStatusConstPtr &msg)
{
    topic_action_cnt_list_[5] = 3;
    if (msg->in_error.val == 0)
    {
        topic_action_cnt_list_[6] = 3;
    }
    if (msg->drives_powered.val == 1)
    {
        topic_action_cnt_list_[7] = 3;
    }
}

int StatusMonitor::checkNodeStatus()
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);
    for (int i = 0; i < node_list_.size(); i++)
    {
        for (int j = 0; j < nodes.size(); j++)
        {
            if (node_list_[i] == nodes[j])
            {
                node_action_list_[i] = true;
                break;
            }
            if (j == nodes.size() - 1)
            {
                node_action_list_[i] = false;
            }
        }
    }
}

void StatusMonitor::readTxt()
{
    std::string path = ros::package::getPath("status_monitor");
    std::string file_name;
    nh_->param("node_file", file_name, std::string("nodes.txt"));
    path = path + "/cfg/" + file_name;
    readNodeList(path);
    node_action_list_.resize(node_list_.size());
}

int StatusMonitor::readNodeList(const std::string &path)
{
    std::ifstream fin(path.c_str(), std::ios::in);
    if (!fin)
    {
        return -1;
    }
    char line[1024] = {0};
    std::string node;
    while (fin.getline(line, sizeof(line)))
    {
        std::stringstream ss(line);
        ss >> node;
        node_list_.push_back(node);
    }
    nh_->setParam("/status/node_list", node_list_);
    fin.clear();
    fin.close();
    return 0;
}

int StatusMonitor::setParam(std::vector<std::string> &nodes, std::vector<bool> &isAction)
{
    std::string prefix = "/status";
    for (int i = 0; i < nodes.size(); i++)
    {
        std::string param_name;
        param_name = prefix + nodes[i];
        nh_->setParam(param_name, isAction[i]);
    }
    return 0;
}
