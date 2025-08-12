#ifndef DDS_BRIDGE_H
#define DDS_BRIDGE_H

#include "MotorController.h"
#include "CanMotorController.h"
#include "ImuSharedData.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <memory>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"

class DDSBridge
{
public:
    DDSBridge(ImuSharedData* imuDataArr[5]);
    ~DDSBridge();

    void LowCmdHandler(const void* msg);
    void PublishLowState();
    void Run();

private:
    ImuSharedData* imuDataArr_[5];
    ThreadPtr lowStatePuberThreadPtr;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_suber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_puber_;
};

#endif // TEST_BRIDGE_H