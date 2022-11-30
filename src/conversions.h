#ifndef _DEF_OCULUS_ROS_CONVERSIONS_H_
#define _DEF_OCULUS_ROS_CONVERSIONS_H_

#include <oculus_driver/Oculus.h>

#include <sensor_msgs/Image.h>

#include <oculus_sonar/OculusHeader.h>
#include <oculus_sonar/OculusVersionInfo.h>
#include <oculus_sonar/OculusStatus.h>
#include <oculus_sonar/OculusFireConfig.h>
#include <oculus_sonar/OculusPing.h>

#include <oculus_sonar/Raw.h>
#include <oculus_sonar/Ping.h>

namespace oculus {

inline ros::Time to_ros_stamp(const SonarDriver::TimePoint& stamp)
{
    size_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
    size_t seconds = nano / 1000000000;
    return ros::Time(seconds, nano - 1000000000*seconds);
}


inline void copy_to_ros(oculus_sonar::OculusHeader& msg, const OculusMessageHeader& header)
{
    msg.oculusId     = header.oculusId;
    msg.srcDeviceId  = header.srcDeviceId;
    msg.dstDeviceId  = header.dstDeviceId;
    msg.msgId        = header.msgId;
    msg.msgVersion   = header.msgVersion;
    msg.payloadSize  = header.payloadSize;
    msg.spare2       = header.spare2;
}

inline void copy_to_ros(oculus_sonar::OculusVersionInfo& msg, const OculusVersionInfo& version)
{
    msg.firmwareVersion0 = version.firmwareVersion0;
    msg.firmwareDate0    = version.firmwareDate0;
    msg.firmwareVersion1 = version.firmwareVersion1;
    msg.firmwareDate1    = version.firmwareDate1;
    msg.firmwareVersion2 = version.firmwareVersion2;
    msg.firmwareDate2    = version.firmwareDate2;
}

inline void copy_to_ros(oculus_sonar::OculusStatus& msg, const OculusStatusMsg& status)
{
    copy_to_ros(msg.hdr, status.hdr);

    msg.deviceId        = status.deviceId;
    msg.deviceType      = status.deviceType;
    msg.partNumber      = status.partNumber;
    msg.status          = status.status;

    copy_to_ros(msg.versinInfo,status.versinInfo);

    msg.ipAddr          = status.ipAddr;
    msg.ipMask          = status.ipMask;
    msg.connectedIpAddr = status.connectedIpAddr;

    msg.macAddr0        = status.macAddr0;
    msg.macAddr1        = status.macAddr1;
    msg.macAddr2        = status.macAddr2;
    msg.macAddr3        = status.macAddr3;
    msg.macAddr4        = status.macAddr4;
    msg.macAddr5        = status.macAddr5;

    msg.temperature0    = status.temperature0;
    msg.temperature1    = status.temperature1;
    msg.temperature2    = status.temperature2;
    msg.temperature3    = status.temperature3;
    msg.temperature4    = status.temperature4;
    msg.temperature5    = status.temperature5;
    msg.temperature6    = status.temperature6;
    msg.temperature7    = status.temperature7;
    msg.pressure        = status.pressure;
}

inline void copy_to_ros(oculus_sonar::OculusFireConfig& msg, const OculusSimpleFireMessage& fireConfig)
{
    copy_to_ros(msg.head, fireConfig.head);

    msg.masterMode      = fireConfig.masterMode;
    msg.pingRate        = fireConfig.pingRate;
    msg.networkSpeed    = fireConfig.networkSpeed;
    msg.gammaCorrection = fireConfig.gammaCorrection;
    msg.flags           = fireConfig.flags;
    msg.range           = fireConfig.range;
    msg.gainPercent     = fireConfig.gainPercent;
    msg.speedOfSound    = fireConfig.speedOfSound;
    msg.salinity        = fireConfig.salinity;
}

inline void copy_to_ros(oculus_sonar::OculusPing& msg, const OculusSimplePingResult& ping)
{
    copy_to_ros(msg.fireMessage, ping.fireMessage);
    msg.pingId            = ping.pingId;
    msg.status            = ping.status;
    msg.frequency         = ping.frequency;
    msg.temperature       = ping.temperature;
    msg.pressure          = ping.pressure;
    msg.speeedOfSoundUsed = ping.speeedOfSoundUsed;
    msg.pingStartTime     = ping.pingStartTime;
    msg.dataSize          = ping.dataSize;
    msg.rangeResolution   = ping.rangeResolution;
    msg.nRanges           = ping.nRanges;
    msg.nBeams            = ping.nBeams;
    msg.imageOffset       = ping.imageOffset;
    msg.imageSize         = ping.imageSize;
    msg.messageSize       = ping.messageSize;
}

inline void copy_to_ros(oculus_sonar::Raw& rosMsg, const oculus::Message::ConstPtr& msg)
{
    rosMsg.header.stamp    = to_ros_stamp(msg->timestamp());
    rosMsg.header.frame_id = "oculus_sonar";
    rosMsg.data = msg->data();
}

inline void copy_to_ros(sensor_msgs::Image& msg, const oculus::PingMessage::ConstPtr& ping)
{
    msg.header.stamp    = to_ros_stamp(ping->timestamp());
    msg.header.frame_id = "oculus_sonar";

    msg.height = ping->range_count();
    
    auto sampleSize = ping->sample_size();
    if(sampleSize == 1) {
        msg.encoding = "mono8";
        if(ping->has_gains())
            msg.width = 4 + ping->bearing_count();
        else
            msg.width = ping->bearing_count();
    }
    else if(sampleSize == 2) {
        msg.encoding = "mono16";
        if(ping->has_gains())
            msg.width = 2 + ping->bearing_count();
        else
            msg.width = ping->bearing_count();
    }
    else {
        std::cerr << "Unhandled ping sample size : " << sampleSize
                  << ". Ping data will have to be parsed by hand." << std::endl;
        msg.encoding = "mono8";
        msg.width    = ping->step();
    }
    
    msg.is_bigendian = 0;
    msg.step = ping->step();
    msg.data.assign(ping->ping_data(),
                    ping->ping_data() + ping->data_size());
}

inline void copy_to_ros(oculus_sonar::Ping& msg, const oculus::PingMessage::ConstPtr& ping)
{
    msg.header.stamp    = to_ros_stamp(ping->timestamp());
    msg.header.frame_id = "oculus_sonar";

    msg.pingId            = ping->ping_index();
    msg.pingFiringDate    = ping->ping_firing_date();
    msg.range             = ping->range();
    msg.gainPercent       = ping->gain_percent();
    msg.frequency         = ping->frequency();
    msg.speedOfSoundUsed  = ping->speed_of_sound_used();
    msg.rangeResolution   = ping->range_resolution();
    msg.temperature       = ping->temperature();
    msg.pressure          = ping->pressure();
    msg.masterMode        = ping->master_mode();
    msg.hasGains          = ping->has_gains();
    msg.nRanges           = ping->range_count();
    msg.nBeams            = ping->bearing_count();
    msg.step              = ping->step();
    msg.sampleSize        = ping->sample_size();

    msg.bearings.assign(ping->bearing_data(),
                        ping->bearing_data() + ping->bearing_count());
    msg.pingData.assign(ping->ping_data(),
                        ping->ping_data() + ping->data_size());
}

} //namespace oculus

#endif //_DEF_OCULUS_ROS_CONVERSIONS_H_
