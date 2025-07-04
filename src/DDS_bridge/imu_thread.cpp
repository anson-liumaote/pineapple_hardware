#include "ImuSharedData.h"
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <atomic>

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5)
        : m_maxNumberOfPacketsInBuffer(maxBufferSize)
        , m_numberOfPacketsInBuffer(0)
    {}

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        xsens::Lock locky(&m_mutex);
        if (!packet) return;
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();
        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
    }
private:
    mutable xsens::Mutex m_mutex;
    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};

void imuThreadFunc(std::atomic<bool>& running, ImuSharedData* imuData)
{
    XsControl* control = XsControl::construct();
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();
    XsPortInfo mtPort;
    for (auto const &portInfo : portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            mtPort = portInfo;
            break;
        }
    }
    if (mtPort.empty()) {
        std::cerr << "No MTi device found. IMU thread exiting." << std::endl;
        return;
    }
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) {
        std::cerr << "Could not open IMU port. IMU thread exiting." << std::endl;
        return;
    }
    XsDevice* device = control->device(mtPort.deviceId());
    CallbackHandler callback;
    device->addCallbackHandler(&callback);
    if (!device->gotoConfig()) return;
    device->readEmtsAndDeviceConfiguration();
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
    if (!device->setOutputConfiguration(configArray)) return;
    if (!device->gotoMeasurement()) return;

    while (running) {
        if (callback.packetAvailable()) {
            XsDataPacket packet = callback.getNextPacket();
            std::lock_guard<std::mutex> lock(imuData->mtx);
            if (packet.containsOrientation()) {
                XsQuaternion q = packet.orientationQuaternion();
                imuData->quaternion[0] = q.w();
                imuData->quaternion[1] = q.x();
                imuData->quaternion[2] = q.y();
                imuData->quaternion[3] = q.z();
                XsEuler euler = packet.orientationEuler();
                imuData->rpy[0] = euler.roll();
                imuData->rpy[1] = euler.pitch();
                imuData->rpy[2] = euler.yaw();
            }
            if (packet.containsCalibratedData()) {
                XsVector acc = packet.calibratedAcceleration();
                XsVector gyr = packet.calibratedGyroscopeData();
                for (int i = 0; i < 3; ++i) {
                    imuData->accel[i] = acc[i];
                    imuData->gyro[i] = gyr[i];
                }
            }
        }
        XsTime::msleep(1);
    }
    control->closePort(mtPort.portName().toStdString());
    control->destruct();
}

Journaller* gJournal = 0;