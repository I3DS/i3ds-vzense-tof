///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __VZENSE_WRAPPER_HPP
#define __VZENSE_WRAPPER_HPP

#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>

#include "Vzense_api2.h"
#include "vzense_enum2str.hpp"

inline bool initialize_vzense() {
  PsReturnStatus status = Ps2_Initialize();

  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "PsInitialize failed!";
    return false;
  }
  BOOST_LOG_TRIVIAL(debug) << "Ps initialized.";
  return true;
}

inline void print_camera_parameters(PsDeviceHandle deviceHandle, uint32_t sessionIndex) {
  PsCameraParameters cameraParameters;
  auto status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &cameraParameters);

  BOOST_LOG_TRIVIAL(info) << "Get PsGetCameraParameters status: " << status;
  BOOST_LOG_TRIVIAL(info) << "Depth Camera Intinsic: ";
  BOOST_LOG_TRIVIAL(info) << "Fx: " << cameraParameters.fx;
  BOOST_LOG_TRIVIAL(info) << "Cx: " << cameraParameters.cx;
  BOOST_LOG_TRIVIAL(info) << "Fy: " << cameraParameters.fy;
  BOOST_LOG_TRIVIAL(info) << "Cy: " << cameraParameters.cy;
  BOOST_LOG_TRIVIAL(info) << "Depth Distortion Coefficient: ";
  BOOST_LOG_TRIVIAL(info) << "K1: " << cameraParameters.k1;
  BOOST_LOG_TRIVIAL(info) << "K2: " << cameraParameters.k2;
  BOOST_LOG_TRIVIAL(info) << "P1: " << cameraParameters.p1;
  BOOST_LOG_TRIVIAL(info) << "P2: " << cameraParameters.p2;
  BOOST_LOG_TRIVIAL(info) << "K3: " << cameraParameters.k3;
  BOOST_LOG_TRIVIAL(info) << "K4: " << cameraParameters.k4;
  BOOST_LOG_TRIVIAL(info) << "K5: " << cameraParameters.k5;
  BOOST_LOG_TRIVIAL(info) << "K6: " << cameraParameters.k6;
}

inline void log_device_info(PsDeviceInfo pDeviceListInfo[], int deviceCount) {
  for (int i = 0; i < deviceCount; i++) {
    BOOST_LOG_TRIVIAL(debug) << "Camera " << i << ":";
    BOOST_LOG_TRIVIAL(debug) << "  Device type:" << pDeviceListInfo[i].devicetype;
    BOOST_LOG_TRIVIAL(debug) << "  URI:        " << pDeviceListInfo[i].uri;
    BOOST_LOG_TRIVIAL(debug) << "  FW:         " << pDeviceListInfo[i].fw;
    BOOST_LOG_TRIVIAL(debug) << "  Alias:      " << pDeviceListInfo[i].alias;
    BOOST_LOG_TRIVIAL(debug) << "  Status:     " << connectStatus2string(pDeviceListInfo[i].status);
  }
}

inline const char* find_device_uri(std::string camera_name, PsDeviceInfo pDeviceListInfo[], int deviceCount) {
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
    for (int i = 0; i < deviceCount; i++) {
      if (pDeviceListInfo[i].status == Connected) {
        return pDeviceListInfo[i].uri;
      }
    }
    return pDeviceListInfo[0].uri;
  }

  // Check if the name is a likely index
  try {
    auto index = boost::lexical_cast<uint8_t>(camera_name);
    if (index < deviceCount) {
      return pDeviceListInfo[index].uri;
    }
  } catch (boost::bad_lexical_cast&) {
    // Could not convert to number, likely not an index.
  }

  // Finally, look for camera based on serial
  for (int i = 0; i < deviceCount; i++) {
    if (camera_name == pDeviceListInfo[i].alias) {
      return pDeviceListInfo[i].uri;
    }
  }

  return nullptr;
}

inline bool connect_to_device(std::string camera_name, PsDeviceHandle *deviceHandle) {
  uint32_t deviceCount = 0;
  PsReturnStatus status = Ps2_GetDeviceCount(&deviceCount);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "PsGetDeviceCount failed: " << returnStatus2string(status);
    return false;
  }
  BOOST_LOG_TRIVIAL(debug) << "Found " << deviceCount << " devices.";
  if (0 == deviceCount) {
    return false;
  }

  PsDeviceInfo pDeviceListInfo[deviceCount];
  status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "Could not get device info: " << returnStatus2string(status);
    return false;
  }

  const char* uri = find_device_uri(camera_name, pDeviceListInfo, deviceCount);
  if (uri == nullptr) {
    BOOST_LOG_TRIVIAL(error) << "Could not find camera: '" << camera_name << "'";
    return false;
  }

  BOOST_LOG_TRIVIAL(info) << "Opening camera: " << uri;
  status = Ps2_OpenDevice(uri, deviceHandle);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "Failed when opening device " << uri;
    return false;
  }

  BOOST_LOG_TRIVIAL(info) << "Device opened." << uri;

  return true;
}

inline bool print_available_cameras() {
	PsReturnStatus ret = Ps2_Initialize();
	if (ret != PsReturnStatus::PsRetOK) {
        BOOST_LOG_TRIVIAL(error) << "PsInitialize FAILED with return value: " << ret;
		return false;
	}

	unsigned int device_count = 0;
    ret = Ps2_GetDeviceCount(&device_count);
    if (ret != PsReturnStatus::PsRetOK) {
        BOOST_LOG_TRIVIAL(error) << "PsGetDeviceCount FAILED with return value: " << ret;
        return false;
    }

    BOOST_LOG_TRIVIAL(info) << "Available devices: " << device_count;
    if (device_count == 0) {
        return true;
    }

    PsDeviceInfo pDeviceListInfo[device_count];
	ret = Ps2_GetDeviceListInfo(pDeviceListInfo, device_count);

    for (unsigned int i = 0; i < device_count; ++i) {
        BOOST_LOG_TRIVIAL(info) << "Info about camera: " << i;
        BOOST_LOG_TRIVIAL(info) << "SessionCount: " << pDeviceListInfo[i].SessionCount;
        BOOST_LOG_TRIVIAL(info) << "devicetype: " << pDeviceListInfo[i].devicetype; // Device model
        BOOST_LOG_TRIVIAL(info) << "uri: " << pDeviceListInfo[i].uri; // /dev file
        BOOST_LOG_TRIVIAL(info) << "fw: " << pDeviceListInfo[i].fw; // Unused?
        BOOST_LOG_TRIVIAL(info) << "Serial#: " << pDeviceListInfo[i].alias; // Serial number
        //BOOST_LOG_TRIVIAL(info) << "status: " << connectStatus2string(pDeviceListInfo[i].status);
    }
    return true;
}

#endif  // __VZENSE_WRAPPER_HPP
