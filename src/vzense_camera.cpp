///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include "vzense_camera.hpp"

#ifndef BOOST_LOG_DYN_LINK
#define BOOST_LOG_DYN_LINK
#endif // BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

namespace logging = boost::log;

int initialize_vzense() {
    PsReturnStatus status = Ps2_Initialize();

    if (status != PsReturnStatus::PsRetOK)
	{
		BOOST_LOG_TRIVIAL(error) << "PsInitialize failed!";
		return -1;
	}
    BOOST_LOG_TRIVIAL(debug) << "Ps initialized.";
    return 1;
}

const char* connectStatus2string(PsConnectStatus status) {
  switch (status) {
    case ConnectUNKNOWN:
      return "Unknown";
    case Unconnected:
      return "Unconnected";
    case Connected:
      return "Connected";
    case Opened:
      return "Opened";
    default:
      return "Undefined";
  }
}

void log_device_info(PsDeviceInfo pDeviceListInfo[], int deviceCount) {
    for (BOOST_TYPEOF(deviceCount) i=0; i<deviceCount; i++) {
        BOOST_LOG_TRIVIAL(debug) << "Camera " << i << ":";
        BOOST_LOG_TRIVIAL(debug) << "  Device type:" << pDeviceListInfo[i].devicetype;
        BOOST_LOG_TRIVIAL(debug) << "  URI:        " << pDeviceListInfo[i].uri;
        BOOST_LOG_TRIVIAL(debug) << "  FW:         " << pDeviceListInfo[i].fw;
        BOOST_LOG_TRIVIAL(debug) << "  Alias:      " << pDeviceListInfo[i].alias;
        BOOST_LOG_TRIVIAL(debug) << "  Status:     " << connectStatus2string(pDeviceListInfo[i].status);
    }
}

const char* find_device_uri(std::string camera_name, PsDeviceInfo pDeviceListInfo[], int deviceCount) {
    log_device_info(pDeviceListInfo, deviceCount);

    if (camera_name == "first") {
        return pDeviceListInfo[0].uri;
    }

    if (camera_name == "last") {
        return pDeviceListInfo[deviceCount - 1].uri;
    }

    if (camera_name == "" || camera_name == "any") {
        if (camera_name == "") {
            BOOST_LOG_TRIVIAL(info) << "Camera name not set, opening first available device.";
        }
        for (BOOST_TYPEOF(deviceCount) i=0; i<deviceCount; i++) {
            if(pDeviceListInfo[i].status == Connected) {
                return pDeviceListInfo[i].uri;
            }
        }
        return pDeviceListInfo[0].uri;
    }

    // TODO(sigurdal): Add indexed access? I.e. 4 -> pDeviceListInfo[4].uri

    for (BOOST_TYPEOF(deviceCount) i=0; i<deviceCount; i++) {
        if (camera_name == pDeviceListInfo[i].alias) {
            return pDeviceListInfo[i].uri;
        }
    }

    return nullptr;
}

bool connect_to_device(std::string camera_name, PsDeviceHandle deviceHandle) {
    uint32_t deviceCount = 0;
    PsReturnStatus status = Ps2_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		BOOST_LOG_TRIVIAL(warning) << "PsGetDeviceCount failed! make sure the DCAM is connected";
		return false;
	}
	BOOST_LOG_TRIVIAL(debug) << "Found " << deviceCount << " devices.";
	if (0 == deviceCount)
	{
		return false;
	}

    PsDeviceInfo pDeviceListInfo[deviceCount];
	status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
    if (status != PsReturnStatus::PsRetOK)
	{
        BOOST_LOG_TRIVIAL(error) << "Could not get device info!";
        return false;
    }

    const char* uri = find_device_uri(camera_name, pDeviceListInfo, deviceCount);
    if (uri == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Could not find camera: '" << camera_name << "'";
        return false;
    }

    BOOST_LOG_TRIVIAL(info) << "Opening camera: " << uri;
	status = Ps2_OpenDevice(uri, &deviceHandle);
    if (status != PsReturnStatus::PsRetOK)
	{
        BOOST_LOG_TRIVIAL(error) << "Failed when opening device " << uri;
        return false;
    }

    return true;
}

i3ds::VzenseCamera::VzenseCamera(i3ds_asn1::NodeID node, const Parameters &param) : i3ds::ToFCamera(node), param_(param) {
    initialize_vzense();
    bool connected = false;
    for (int i=0; i<10; i++) {
        if (!connect_to_device(param_.camera_name, deviceHandle_)) {
            BOOST_LOG_TRIVIAL(warning) << "Retrying camera connection in 1s";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        connected = true;
        break;
    }
    if (!connected) {
        BOOST_LOG_TRIVIAL(error) << "Could not connect to camera";
        return;
    }
}


void i3ds::VzenseCamera::do_activate() {
    //TODO(sigurdm): implement
}
