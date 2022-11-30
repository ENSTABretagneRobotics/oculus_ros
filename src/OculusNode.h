#ifndef _OCULUS_SONAR_OCULUS_NODE_H_
#define _OCULUS_SONAR_OCULUS_NODE_H_

#include <ros/ros.h>

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>
#include <oculus_driver/OculusMessage.h>

#include <oculus_sonar/OculusStatus.h>
#include <oculus_sonar/Ping.h>
#include <oculus_sonar/OculusStampedPing.h>

#include <dynamic_reconfigure/server.h>
#include <oculus_sonar/OculusSonarConfig.h>

class OculusNode
{
    protected:

    std::string pingTopic_;
    std::string statusTopic_;
    std::string pingImageTopic_;
    std::string rawTopic_;
    std::string pingTopicDeprecated_;
    bool        publishWithoutSubs_;

    ros::NodeHandle node_;

    ros::Publisher pingPublisher_;
    ros::Publisher statusPublisher_;
    ros::Publisher imagePublisher_;
    ros::Publisher rawPublisher_;
    ros::Publisher pingPublisherDeprecated_;

    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer_;

    oculus::AsyncService service_;
    oculus::SonarDriver  sonar_;

    public:

    OculusNode(const std::string& nodeName);
    ~OculusNode();

    void start();
    void stop();

    void ping_callback(const oculus::PingMessage::ConstPtr& msg);
    void status_callback(const OculusStatusMsg& status);
    void message_callback(const oculus::Message::ConstPtr& msg);

    void reconfigure_callback(oculus_sonar::OculusSonarConfig& config,
                              int32_t level);

    bool has_ping_subscribers() const;
    void dummy_callback(const OculusMessageHeader& msg);

    // this will be removed in future releases
    void publish_deprecated(const oculus::PingMessage::ConstPtr& msg);
};




#endif //_OCULUS_SONAR_OCULUS_NODE_H_
