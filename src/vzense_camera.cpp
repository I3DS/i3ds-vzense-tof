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

#include <i3ds/time.hpp>

#include "vzense_enum2str.hpp"
#include "vzense_helpers.hpp"

namespace logging = boost::log;

i3ds::VzenseCamera::VzenseCamera(Context::Ptr context, i3ds_asn1::NodeID node, const Parameters& param) :
    i3ds::ToFCamera(node),
    param_(param),
    sampler_([this](unsigned int ts){return sample_loop(ts);}),
    publisher_(context, node) 
{
  set_device_name(param.camera_name);
}


double i3ds::VzenseCamera::range_min_depth() {
  PsDepthRange depth_range;
  auto status = Ps2_GetDepthRange(device_handle_, session_index_, &depth_range);
  if (status != PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "Could not get depth range status " << returnStatus2string(status);
    throw i3ds::DeviceError("Could not get depth range status");
  }

  // From datasheet found: http://shop.notavis.com/Dokumente/Vzense_DCAM710.pdf
  switch (depth_range) {
    case PsNearRange:
    case PsXNearRange:
    case PsXXNearRange:
      return 0.35;

    case PsMidRange:
    case PsXMidRange:
    case PsXXMidRange:
      return 0.5;

    case PsFarRange:
    case PsXFarRange:
    case PsXXFarRange:

      return 0.8;
    default:
      BOOST_LOG_TRIVIAL(error) << "Unknown depth range: " << depth_range;
      throw i3ds::DeviceError("Could not get depth range status");
  }
}


double i3ds::VzenseCamera::range_max_depth() {
  PsDepthRange depth_range;
  auto status = Ps2_GetDepthRange(device_handle_, session_index_, &depth_range);
  if (status != PsRetOK) {
    BOOST_LOG_TRIVIAL(error) << "Could not get depth range status " << returnStatus2string(status);
    throw i3ds::DeviceError("Could not get depth range status");
  }

  // From datasheet found: http://shop.notavis.com/Dokumente/Vzense_DCAM710.pdf
  switch (depth_range) {
    case PsNearRange:
    case PsXNearRange:
    case PsXXNearRange:
      return 1.5;
    case PsMidRange:
    case PsXMidRange:
    case PsXXMidRange:
      return 2.8;
    case PsFarRange:
    case PsXFarRange:
    case PsXXFarRange:
      return 4.4;
    default:
      BOOST_LOG_TRIVIAL(error) << "Unknown depth range: " << depth_range;
      throw i3ds::DeviceError("Could not get depth range status");
  }
}


void set_asn_string(i3ds_asn1::T_String& dst, const std::string txt) {
  auto msg_arr = reinterpret_cast<char*>(dst.arr);
  strncpy(msg_arr, txt.c_str(), 40);
  msg_arr[39] = '\0';
  dst.nCount = strlen(msg_arr);
}


void i3ds::VzenseCamera::handle_range(ToFCamera::RangeService::Data& command) {
  BOOST_LOG_TRIVIAL(info) << "Setting range:";
  BOOST_LOG_TRIVIAL(info) << "  min: " << command.request.min_depth;
  BOOST_LOG_TRIVIAL(info) << "  max: " << command.request.max_depth;

  auto req_min = command.request.min_depth;
  auto req_max = command.request.max_depth;

  // Default value of tool, set to zero instead
  if (command.request.max_depth == 13.32) {
    BOOST_LOG_TRIVIAL(info) << "  (zeroed)";
    req_max = 0;
  }

  command.response.result = i3ds_asn1::ResultCode_success;

  // Try to translate from requested min/max depth to PsDepthRange
  PsDepthRange depth_range = PsUnknown;

  if (req_max != 0) {
    if (req_max <= 1.5) {
      depth_range = PsNearRange;
    } else if (req_max <= 2.8) {
      depth_range = PsMidRange;
      if (req_min != 0 && req_min < 0.5) {
        command.response.result = i3ds_asn1::ResultCode_error_unsupported;
      }
    } else if (req_max <= 5) {
      depth_range = PsFarRange;
      if (req_min != 0 && req_min < 0.8) {
        command.response.result = i3ds_asn1::ResultCode_error_unsupported;
      }
    } else {
      command.response.result = i3ds_asn1::ResultCode_error_unsupported;
    }
  } else {
    set_asn_string(command.response.message, "Use max distance to set range.");
    return;
  }

  if (command.response.result != i3ds_asn1::ResultCode_success) {
    BOOST_LOG_TRIVIAL(warning) << "Could not set range - invalid range: [" << command.request.min_depth << ", "
                               << command.request.max_depth << "]";
    set_asn_string(command.response.message, "Valid ranges: .35-1.5, .5-2.8, .8-4.4");
    return;
  }

  wanted_range_ = depth_range;

  // If the stream is started we can run this to update it live. If not, we're done.
  if (state() != i3ds_asn1::SensorState_operational) {
    command.response.result = i3ds_asn1::ResultCode_success;
    return;
  }
  
  auto status = Ps2_SetDepthRange(device_handle_, session_index_, depth_range);
  if (status != PsRetOK) {
    command.response.result = i3ds_asn1::ResultCode_error_other;
    BOOST_LOG_TRIVIAL(error) << "Could not set range to " << depth_range << ": " << returnStatus2string(status);
    set_asn_string(command.response.message, "Restart stream to update range.");
    return;
  }
}


void i3ds::VzenseCamera::do_activate() {
  BOOST_LOG_TRIVIAL(debug) << "Activating Vzense";
  if (!initialize_vzense()) {
    throw i3ds::DeviceError("Could not initialize Vzense driver.");
  }

  bool connected = false;
  uint retries = 0;

  for (uint i = 0; i < retries + 1; i++) {
    if (!connect_to_device(param_.camera_name, &device_handle_)) {
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
  BOOST_LOG_TRIVIAL(info) << "Activated Vzense";
}


void i3ds::VzenseCamera::do_deactivate() {
  BOOST_LOG_TRIVIAL(debug) << "Deactivating Vzense";
  Ps2_CloseDevice(&device_handle_);
  Ps2_Shutdown();
  BOOST_LOG_TRIVIAL(info) << "Deactivated Vzense";
}


void i3ds::VzenseCamera::do_start() {
  BOOST_LOG_TRIVIAL(debug) << "Starting Vzense";
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

  status = Ps2_SetDepthRange(device_handle_, session_index_, wanted_range_);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed to set range: " << returnStatus2string(status);
  }

  PsDepthRange depth_range;
  status = Ps2_GetDepthRange(device_handle_, session_index_, &depth_range);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed to get range: " << returnStatus2string(status);
    Ps2_StopStream(device_handle_, session_index_);
    throw i3ds::DeviceError("Could not get the depth range of the camera.");
  }

  if (depth_range != wanted_range_) {
    BOOST_LOG_TRIVIAL(warning) << "Unexpected depth range. ";
  }

  sampler_.Start(period());
  BOOST_LOG_TRIVIAL(info) << "Started Vzense";
}


bool i3ds::VzenseCamera::sample_loop(i3ds_asn1::Timepoint timestamp) {

  // Handle the frames
  PsFrameReady frameReady = {0};
  auto status = Ps2_ReadNextFrame(device_handle_, session_index_, &frameReady);

  if (status != PsRetOK) {
      BOOST_LOG_TRIVIAL(error) << "Error reading frame: " << returnStatus2string(status);
      return false;
  }

  if (frameReady.depth == 1) {
      PsFrame depthFrame = {0};
      PsReturnStatus status = Ps2_GetFrame(device_handle_, session_index_, PsDepthFrame, &depthFrame);

      if (status == PsRetOK && depthFrame.pFrameData != NULL) {
          send_sample(reinterpret_cast<uint16_t*>(depthFrame.pFrameData), depthFrame.width, depthFrame.height);
      } else {
          BOOST_LOG_TRIVIAL(warning) << "Ps2_GetFrame PsDepthFrame status:" << returnStatus2string(status);
      }
  }

  return true;
}


void i3ds::VzenseCamera::do_stop() {
  BOOST_LOG_TRIVIAL(debug) << "Stopping Vzense";
  sampler_.Stop();
  auto status = Ps2_StopStream(device_handle_, session_index_);

  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Failed stopping stream: " << returnStatus2string(status);
    throw i3ds::DeviceError("Failed to stop stream.");
  }

  BOOST_LOG_TRIVIAL(info) << "Stopped Vzense";
}


void i3ds::VzenseCamera::send_sample(const uint16_t* data, uint width, uint height) {
  ToFCamera::MeasurementTopic::Data frame;
  ToFCamera::MeasurementTopic::Codec::Initialize(frame);

  frame.descriptor.attributes.timestamp = get_timestamp();
  frame.descriptor.attributes.validity = i3ds_asn1::SampleValidity_sample_valid;
  frame.descriptor.width = width;
  frame.descriptor.height = height;

  const uint size = width * height;
  frame.depths.resize(size);
  for (size_t i = 0; i < size; i++) {
    if (data[i] == 0xffff) {
      frame.depths[i] = -1;
      continue;
    }
    frame.depths[i] = static_cast<float>(data[i]) / 1000.0f; // Scale from [mm] to [m]
  }

  publisher_.Send<ToFCamera::MeasurementTopic>(frame);
  update_and_check_batch_count();
}
