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
#endif  // BOOST_LOG_DYN_LINK

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <boost/lexical_cast.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/opencv.hpp>
#include <i3ds/time.hpp>

namespace logging = boost::log;

bool initialize_vzense() {
  PsReturnStatus status = Ps2_Initialize();

  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "PsInitialize failed!";
    return false;
  }
  BOOST_LOG_TRIVIAL(debug) << "Ps initialized.";
  return true;
}

const char* returnStatus2string(PsReturnStatus status) {
  switch (status) {
    case PsRetOK:
      return "OK";
    case PsRetNoDeviceConnected:
      return "NoDeviceConnected";
    case PsRetInvalidDeviceIndex:
      return "InvalidDeviceIndex";
    case PsRetDevicePointerIsNull:
      return "DevicePointerIsNull";
    case PsRetInvalidFrameType:
      return "InvalidFrameType";
    case PsRetFramePointerIsNull:
      return "FramePointerIsNull";
    case PsRetNoPropertyValueGet:
      return "NoPropertyValueGet";
    case PsRetNoPropertyValueSet:
      return "NoPropertyValueSet";
    case PsRetPropertyPointerIsNull:
      return "PropertyPointerIsNull";
    case PsRetPropertySizeNotEnough:
      return "PropertySizeNotEnough";
    case PsRetInvalidDepthRange:
      return "InvalidDepthRange";
    case PsRetReadNextFrameTimeOut:
      return "ReadNextFrameTimeOut";
    case PsRetInputPointerIsNull:
      return "InputPointerIsNull";
    case PsRetCameraNotOpened:
      return "CameraNotOpened";
    case PsRetInvalidCameraType:
      return "InvalidCameraType";
    case PsRetInvalidParams:
      return "InvalidParams";
    case PsRetCurrentVersionNotSupport:
      return "CurrentVersionNotSupport";
    case PsRetUpgradeImgError:
      return "UpgradeImgError";
    case PsRetUpgradeImgPathTooLong:
      return "UpgradeImgPathTooLong";
    case PsRetUpgradeCallbackNotSet:
      return "UpgradeCallbackNotSet";
    case PsRetNoAdapterConnected:
      return "NoAdapterConnected";
    case PsRetReInitialized:
      return "ReInitialized";
    case PsRetNoInitialized:
      return "NoInitialized";
    case PsRetCameraOpened:
      return "CameraOpened";
    case PsRetCmdError:
      return "CmdError";
    case PsRetCmdSyncTimeOut:
      return "CmdSyncTimeOut";
    case PsRetOthers:
      return "Others";
    default:
      return "Undefined";
  }
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

const char* dataMode2str(PsDataMode mode) {
  switch (mode) {
    case PsDepthAndRGB_30:
      return "DepthAndRGB_30";
    case PsIRAndRGB_30:
      return "IRAndRGB_30";
    case PsDepthAndIR_30:
      return "PsDepthAndIR_30";
    case PsNoCCD_30:
      return "NoCCD_30";
    case PsDepthAndIR_15_RGB_30:
      return "DepthAndIR_15_RGB_30";
    case PsWDR_Depth:
      return "WDR_Depth";
    case PsWDR_IR:
      return "PsWDR_IR";
    case PsWDR_DepthAndIR:
      return "PsWDR_DepthAndIR";
    default:
      return "Undefined mode";
  }
}

uint32_t range_to_slope(PsDepthRange depth_range, PsMeasuringRange measuring_range) {
  switch (depth_range) {
    case PsNearRange:
    case PsXNearRange:
    case PsXXNearRange:
      return measuring_range.effectDepthMaxNear;

    case PsMidRange:
    case PsXMidRange:
    case PsXXMidRange:
      return measuring_range.effectDepthMaxMid;

    case PsFarRange:
    case PsXFarRange:
    case PsXXFarRange:

      return measuring_range.effectDepthMaxFar;
    default:
      BOOST_LOG_TRIVIAL(error) << "Could not get slope from range: " << depth_range;
      return 0;
  }
}

void print_camera_parameters(PsDeviceHandle deviceHandle, uint32_t sessionIndex) {
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

void log_device_info(PsDeviceInfo pDeviceListInfo[], int deviceCount) {
  for (BOOST_TYPEOF(deviceCount) i = 0; i < deviceCount; i++) {
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
    for (BOOST_TYPEOF(deviceCount) i = 0; i < deviceCount; i++) {
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
  for (BOOST_TYPEOF(deviceCount) i = 0; i < deviceCount; i++) {
    if (camera_name == pDeviceListInfo[i].alias) {
      return pDeviceListInfo[i].uri;
    }
  }

  return nullptr;
}

bool connect_to_device(std::string camera_name, PsDeviceHandle& deviceHandle) {
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
  status = Ps2_OpenDevice(uri, &deviceHandle);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "Failed when opening device " << uri;
    return false;
  }

  return true;
}

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg) {
  dispImg = cv::Mat(height, width, CV_16UC1, pData);
  cv::Point2d pointxy(width / 2, height / 2);
  int val = dispImg.at<ushort>(pointxy);
  char text[20];
#ifdef _WIN32
  sprintf_s(text, "%d", val);
#else
  snprintf(text, sizeof(text), "%d", val);
#endif
  dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
  applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
  int color;
  if (val > 2500)
    color = 0;
  else
    color = 4096;
  circle(dispImg, pointxy, 4, cv::Scalar(color, color, color), -1, 8, 0);
  putText(dispImg, text, pointxy, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(color, color, color));
}
bool i3ds::VzenseCamera::get_depth_frame() {
  PsFrame depthFrame = {0};
  PsReturnStatus status = Ps2_GetFrame(device_handle_, session_index_, PsDepthFrame, &depthFrame);

  if (status != PsRetOK || depthFrame.pFrameData == NULL) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_GetFrame PsDepthFrame status:" << returnStatus2string(status);
    return false;
  }

  cv::Mat imageMat;
  Opencv_Depth(slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);

  cv::imshow("Depth", imageMat);

  send_sample(reinterpret_cast<uint16_t*>(depthFrame.pFrameData), depthFrame.width, depthFrame.height);
  return true;
}

i3ds::VzenseCamera::VzenseCamera(Context::Ptr context, i3ds_asn1::NodeID node, const Parameters& param)
    : i3ds::ToFCamera(node), param_(param), publisher_ (context, node ) {}

void i3ds::VzenseCamera::do_activate() {
  if (!initialize_vzense()) {
    throw i3ds::DeviceError("Could not initialize Vzense driver.");
  }

  bool connected = false;
  uint retries = 0;

  for (uint i = 0; i < retries + 1; i++) {
    if (!connect_to_device(param_.camera_name, device_handle_)) {
      BOOST_LOG_TRIVIAL(warning) << "Retrying camera connection in 1s";
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    connected = true;
    break;
  }

  if (!connected) {
    BOOST_LOG_TRIVIAL(error) << "Could not connect to camera";
    throw i3ds::DeviceError("Could not connect to camera.");
  }
}

void i3ds::VzenseCamera::do_deactivate() {
  Ps2_CloseDevice(&device_handle_);
  Ps2_Shutdown();
  cv::destroyAllWindows();
}

void i3ds::VzenseCamera::do_start() {
  session_index_ = 0;

  auto status = Ps2_StartStream(device_handle_, session_index_);
  
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "StartStream failed: " << returnStatus2string(status);
    throw i3ds::DeviceError("Failed to start stream.");
  }

  // TODO(sigurdal): Implement WDR mode?
  // PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
  auto dataMode = PsDepthAndIR_30;
  status = Ps2_GetDataMode(device_handle_, session_index_, &dataMode);

  if (status != PsReturnStatus::PsRetOK)
    BOOST_LOG_TRIVIAL(warning) << "Ps2_GetDataMode failed!";
  else
    BOOST_LOG_TRIVIAL(info) << "Get Ps2_GetDataMode: " << dataMode2str(dataMode);

  if (dataMode == PsWDR_Depth) {
    // TODO(sigurdal): Implement WDR mode?
    BOOST_LOG_TRIVIAL(error) << "Wide Dynamic Range (WDR) not implemented yet. Aborting.";
    Ps2_StopStream(device_handle_, session_index_);
    throw i3ds::DeviceError("Wide Dynamic Range mode is not implemented yet. Aborting.");
  }

  PsDepthRange depth_range = PsNearRange;

  status = Ps2_SetDepthRange(device_handle_, session_index_, PsFarRange);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed to set range: " << returnStatus2string(status);
  }

  status = Ps2_GetDepthRange(device_handle_, session_index_, &depth_range);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed to get range: " << returnStatus2string(status);
    Ps2_StopStream(device_handle_, session_index_);
    throw i3ds::DeviceError("Could not get the depth range of the camera.");
  }

  // Get required parameters
  PsMeasuringRange measuringrange = {0};
  status = Ps2_GetMeasuringRange(device_handle_, session_index_, depth_range, &measuringrange);

  slope_ = range_to_slope(depth_range, measuringrange);
  BOOST_LOG_TRIVIAL(info) << "Slope is: " << slope_;

  sampler_ = std::thread(&i3ds::VzenseCamera::sample_loop, this);
}

void i3ds::VzenseCamera::sample_loop() {
  sampler_running_ = true;

  // Handle the frames
  PsFrameReady frameReady = {0};
  while (sampler_running_) {
    auto status = Ps2_ReadNextFrame(device_handle_, session_index_, &frameReady);

    if (status != PsRetOK) {
      BOOST_LOG_TRIVIAL(error) << "Error reading frame: " << returnStatus2string(status);
      break;
    }

    if (1 == frameReady.depth) {
      get_depth_frame();
    }

    unsigned char key = cv::waitKey(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

    switch (key) {
      case 'q':
      case 'Q':
        running = false;
        break;
    }
void i3ds::VzenseCamera::send_sample(const uint16_t* data, uint width, uint height) {
  ToFCamera::MeasurementTopic::Data frame;
  ToFCamera::MeasurementTopic::Codec::Initialize(frame);

  frame.descriptor.attributes.timestamp = get_timestamp();

  frame.descriptor.width = width;
  frame.descriptor.height = height;
  const uint size = width * height;
  frame.depths.resize(size);
  for (size_t i = 0; i < size; i++) {
    frame.depths[i] = static_cast<float>(data[i]);
  }

  publisher_.Send<ToFCamera::MeasurementTopic> (frame);
}

void i3ds::VzenseCamera::do_stop() {
  sampler_running_ = false;
  auto status = Ps2_StopStream(device_handle_, session_index_);

  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed stopping stream: " << returnStatus2string(status);
    throw i3ds::DeviceError("Failed to stop stream.");
  }

  sampler_.join();
}
