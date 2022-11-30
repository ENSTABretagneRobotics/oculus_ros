#include <ros/ros.h>

#include "OculusNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_sonar");

    //// Setting up namespace to node name (why not by default ??)
    //std::string nodeName = ros::this_node::getName();
    //if(nodeName[0] == '/') {
    //    nodeName = nodeName.substr(1, nodeName.size() - 1);
    //}

    OculusNode sonarNode("oculus_sonar");
    ros::spin();

    return 0;
}
