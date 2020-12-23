#include "status_monitor/status_monitor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_monitor");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(2);
    spin.start();
    StatusMonitor s(&nh);
    ros::waitForShutdown();
    return 0;
}