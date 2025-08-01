
//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  


//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mti6x0device.h"

#include <xstypes/xsdatapacket.h>
#include "replyobject.h"
#include "communicator.h"
#include "scenariomatchpred.h"
#include "messageserializer.h"
#include <xstypes/xsstatusflag.h>

#include <xstypes/xssensorranges.h>
#include "synclinegmt.h"
#include <xstypes/xssyncsetting.h>
#include <xstypes/xscanoutputconfigurationarray.h>
#include <xstypes/xsstringoutputtypearray.h>
#include <xstypes/xsintarray.h>
#include "xsgnssreceivertype.h"

using namespace xsens;

/*! \brief Constructs a device
	\param comm The communicator to construct with
*/
Mti6X0Device::Mti6X0Device(Communicator* comm) :
	MtiBaseDeviceEx(comm)
{
}

/*! \brief Destroys a device
*/
Mti6X0Device::~Mti6X0Device()
{
}

/*! \brief Returns the base update rate (Hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult Mti6X0Device::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	MtiBaseDevice::BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType == XDI_FreeAcceleration && deviceId().isImu()) ||
		((dataType & XDI_FullTypeMask) == XDI_LocationId) ||
		((dataType & XDI_FullTypeMask) == XDI_DeviceId) ||
		((dataType & XDI_FullTypeMask) == XDI_RawGyroTemp))
		return result;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR)
	{
		result.m_frequency = 2000;
		return result;
	}

	if ((dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		result.m_frequency = hardwareVersion().major() == 2 ? 2000 : 1600;
		return result;
	}

	auto baseFreq = [this](XsDataIdentifier dataType)
	{
		XsDataIdentifier fullType = (dataType & XDI_FullTypeMask);
		switch (dataType & XDI_TypeMask)
		{
			case XDI_None:
				return 400;
			case XDI_TimestampGroup:
				return XDI_MAX_FREQUENCY_VAL;
			case XDI_StatusGroup:
				return 400;
			case XDI_TemperatureGroup:
				return 400;
			case XDI_OrientationGroup:
				return deviceId().isImu() ? 0 : 400;
			case XDI_AccelerationGroup:
				return 400;
			case XDI_AngularVelocityGroup:
				return 400;
			case XDI_MagneticGroup:
				return 100;
			case XDI_RawSensorGroup:
				return 200;

			case XDI_GnssGroup:
				if (fullType == XDI_GnssPvtPulse)
					return deviceId().isRtk() ? XDI_MAX_FREQUENCY_VAL : 0;
				return deviceId().isGnss() ? XDI_MAX_FREQUENCY_VAL : 0;
			case XDI_PressureGroup:
				return 100;
			case XDI_PositionGroup:
				return deviceId().isGnss() ? 400 : 0;
			case XDI_VelocityGroup:
				return deviceId().isGnss() ? 400 : 0;
			default:
				return 0;
		}
	};
	result.m_frequency = baseFreq(dataType);

	if (((dataType & XDI_TypeMask) == XDI_TimestampGroup) || ((dataType & XDI_TypeMask) == XDI_GnssGroup))
		result.m_divedable = false;

	return result;
}

/*! \returns The sync line for a mtitx0 device This overrides the base class method.
	\param setting The sync setting to get a sync line from
*/
uint8_t Mti6X0Device::syncLine(const XsSyncSetting& setting) const
{
	SyncLineGmt gmtLine = xslToXslgmt(setting.m_line);
	assert(gmtLine != XSLGMT_Invalid);
	return static_cast<uint8_t>(gmtLine);
}

/*! \returns True if this device has an ICC support
*/
bool Mti6X0Device::hasIccSupport() const
{
	return (firmwareVersion() >= XsVersion(1, 1, 0));
}


XsStringOutputTypeArray Mti6X0Device::supportedStringOutputTypes() const
{
	XsStringOutputTypeArray outputs;

	outputs.push_back(XSOT_PSONCMS);
	outputs.push_back(XSOT_HCMTW);
	outputs.push_back(XSOT_HEROT);
	outputs.push_back(XSOT_PTCF);
	outputs.push_back(XSOT_GPZDA);
	outputs.push_back(XSOT_TSS2);
	outputs.push_back(XSOT_PHTRO);
	outputs.push_back(XSOT_PRDID);
	outputs.push_back(XSOT_EM1000);
	outputs.push_back(XSOT_HEHDT);
	outputs.push_back(XSOT_HCHDM);
	outputs.push_back(XSOT_GPGGA);
	outputs.push_back(XSOT_GPRMC);
	outputs.push_back(XSOT_XSVEL);
	outputs.push_back(XSOT_HCHDG);

	return outputs;
}

/*! \brief Sets the string output mode for this device
	\param type The type to set
	\param frequency The frequency to set
	\returns True if the device was successfully updated
*/
bool Mti6X0Device::setStringOutputMode6x0(uint32_t type, uint16_t frequency)
{
	XsMessage sndType(XMID_SetStringOutputConfig);
	sndType.setBusId(XS_BID_MASTER);	// Always send to master device
	sndType.setDataLong(type);
	sndType.setDataShort(frequency, 4);

	if (!doTransaction(sndType))
		return false;

	return true;
}

uint32_t Mti6X0Device::supportedStatusFlags() const
{
	return (uint32_t)(
			XSF_ExternalClockSynced
			| (deviceId().isImu() ? 0 : XSF_OrientationValid
				| XSF_NoRotationMask
				| XSF_RepresentativeMotion
			)
			| (deviceId().isGnss() ? XSF_GpsValid : 0)
			| XSF_ClipAccX
			| XSF_ClipAccY
			| XSF_ClipAccZ
			| XSF_ClipGyrX
			| XSF_ClipGyrY
			| XSF_ClipGyrZ
			| XSF_ClipMagX
			| XSF_ClipMagY
			| XSF_ClipMagZ
			//|XSF_Retransmitted
			| XSF_ClippingDetected
			//|XSF_Interpolated
			| XSF_SyncIn
			| XSF_SyncOut
			| (deviceId().isGnss() ? XSF_FilterMode : 0)
			| (deviceId().isGnss() ? XSF_HaveGnssTimePulse : 0)
			| (deviceId().isRtk() ? XSF_RtkStatus : 0)
		);
}

/*! \copybrief XsDevice::shortProductCode
*/
XsString Mti6X0Device::shortProductCode() const
{
	return stripProductCode(deviceId());
}

/*! \copybrief XsDevice::canOutputConfiguration
*/
XsCanOutputConfigurationArray Mti6X0Device::canOutputConfiguration() const
{
	XsMessage snd(XMID_ReqCanOutputConfig), rcv;
	if (!doTransaction(snd, rcv))
		return XsCanOutputConfigurationArray();

	XsCanOutputConfigurationArray config;
	MessageDeserializer serializer(rcv);
	serializer >> config;

	return config;
}

/*! \copydoc XsDevice::setCanOutputConfiguration
*/
bool Mti6X0Device::setCanOutputConfiguration(XsCanOutputConfigurationArray& config)
{
	XsMessage snd(XMID_SetCanOutputConfig, 4);
	snd.setBusId(busId());
	bool wasEmpty = config.empty();
	MessageSerializer(snd) << config;

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	MessageDeserializer(rcv) >> config;
	if (wasEmpty && config.size() == 1 && config[0] == XsCanOutputConfiguration(XCFF_11Bit_Identifier, XCDI_Invalid, 0, 0))
		config.clear();
	return true;
}

/*! \copybrief XsDevice::canConfiguration
*/
uint32_t Mti6X0Device::canConfiguration() const
{
	XsMessage snd(XMID_ReqCanConfig), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataLong();
}

/*! \copydoc XsDevice::portConfiguration
*/
XsIntArray Mti6X0Device::portConfiguration() const
{
	XsMessage snd(XMID_ReqPortConfig), rcv;
	if (!doTransaction(snd, rcv))
		return XsIntArray();

	XsIntArray rv;
	rv.push_back((int)rcv.getDataLong(0));
	rv.push_back((int)rcv.getDataLong(4));
	rv.push_back((int)rcv.getDataLong(8));
	return rv;
}

/*! \copydoc XsDevice::setPortConfiguration
*/
bool Mti6X0Device::setPortConfiguration(XsIntArray& config)
{
	XsIntArray currentConfig = portConfiguration();

	XsMessage snd(XMID_SetPortConfig);
	snd.setBusId(busId());

	if (config.size() > 0)
		snd.setDataLong((uint32_t)config[0], 0);
	if (config.size() > 1)
		snd.setDataLong((uint32_t)config[1], 4);
	if (config.size() > 2)
		snd.setDataLong((uint32_t)config[2], 8);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	if (currentConfig != config)
		return reset();

	return true;
}

/*! \copydoc XsDevice::setCanOutputConfiguration
*/
bool Mti6X0Device::setCanConfiguration(uint32_t config)
{
	uint32_t currentConfig = canConfiguration();

	XsMessage snd(XMID_SetCanConfig, 4);
	snd.setBusId(busId());
	snd.setDataLong(config);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	if ((currentConfig & 0xFF) != (config & 0xFF))
		return reset();

	return true;
}

/*! \copydoc XsDevice::setGnssLeverArm
*/
bool Mti6X0Device::setGnssLeverArm(const XsVector& arm)
{
	if (!deviceId().isRtk())
		return false;

	XsMessage snd(XMID_SetGnssLeverArm, 3 * sizeof(float));
	snd.setBusId(busId());
	snd.setDataFloat((float)arm[0], 0/* sizeof(float)*/);
	snd.setDataFloat((float)arm[1], 1 * sizeof(float));
	snd.setDataFloat((float)arm[2], 2 * sizeof(float));

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	return true;
}

/*! \copydoc XsDevice::gnssLeverArm
*/
XsVector Mti6X0Device::gnssLeverArm() const
{
	if (!deviceId().isRtk())
		return XsVector();

	XsMessage snd(XMID_ReqGnssLeverArm), rcv;
	if (!doTransaction(snd, rcv))
		return XsVector();

	XsVector arm(3);
	arm[0] = rcv.getDataFloat(0/* sizeof(float)*/);
	arm[1] = rcv.getDataFloat(1 * sizeof(float));
	arm[2] = rcv.getDataFloat(2 * sizeof(float));
	return arm;
}

/*! \copydoc XsDevice::ubloxGnssPlatform
*/
XsUbloxGnssPlatform Mti6X0Device::ubloxGnssPlatform() const
{
	XsUbloxGnssPlatform platform = XGP_Portable;
	auto gnssReceivSett = gnssReceiverSettings();
	if (gnssReceivSett.size() > 3)
	{
		int gnssReceiverModel = gnssReceivSett[0];
		int gnssReceiverOptions = gnssReceivSett[3];

		//only read the value for u-blox devices
		XsGnssReceiverType actualType = (XsGnssReceiverType)gnssReceiverModel;
		if (actualType == XGRT_Ublox_Max_M8Q || actualType == XGRT_Ublox_Neo_M8P || actualType == XGRT_Ublox_ZED_F9P)
			platform = static_cast<XsUbloxGnssPlatform>(gnssReceiverOptions & 0xFFFF);
	}
	return platform;
}

/*! \copydoc XsDevice::setUbloxGnssPlatform
*/
bool Mti6X0Device::setUbloxGnssPlatform(XsUbloxGnssPlatform ubloxGnssPlatform)
{
	bool result = false;
	auto gnssReceivSett = gnssReceiverSettings();
	if (gnssReceivSett.size() > 3)
	{
		int gnssReceiverModel = gnssReceivSett[0];
		int gnssReceiverOptions = gnssReceivSett[3];

		//only set the value for u-blox devices
		XsGnssReceiverType actualType = (XsGnssReceiverType)gnssReceiverModel;
		if (actualType == XGRT_Ublox_Max_M8Q || actualType == XGRT_Ublox_Neo_M8P || actualType == XGRT_Ublox_ZED_F9P)
		{
			gnssReceivSett[3] = (int) ((((unsigned int)gnssReceiverOptions) & 0xFFFF0000U) | uint16_t(ubloxGnssPlatform));
			result = setGnssReceiverSettings(gnssReceivSett);
		}
	}
	return result;
}
