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
#include "DCAM710/Vzense_types_710.h"
#include <cstdint>
#include <i3ds/depthmap.hpp>
#include <i3ds_asn1/Common.hpp>
#include <i3ds_asn1/SampleAttribute.hpp>
#include <memory>

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
#include <ctime>

#include "vzense_enum2str.hpp"
#include "vzense_helpers.hpp"


i3ds::VzenseCamera::VzenseCamera(Context::Ptr context, i3ds_asn1::NodeID node, const Parameters& param) :
    i3ds::ToFCamera(node),
    param_(param),
    sampler_([this](unsigned int ts){return sample_loop(ts);}),
    publisher_(context, node),
    flipped_image_(nullptr)
{
  set_device_name(param.camera_name);
}


double i3ds::VzenseCamera::range_min_depth() const {
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


double i3ds::VzenseCamera::range_max_depth() const {
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


  BOOST_LOG_TRIVIAL(info) << "-------- Camera filters and params:";


  // FILTERS:
  bool depth_correction;
  status = Ps2_SetDepthDistortionCorrectionEnabled(device_handle_, session_index_, param_.filter_depth_distortion);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetDepthDistortionCorrectionEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetDepthDistortionCorrectionEnabled(device_handle_, session_index_, &depth_correction);
    BOOST_LOG_TRIVIAL(info) << "Depth Distortion Correction Filter: " << depth_correction;
  }

  bool ir_correction;
  status = Ps2_SetIrDistortionCorrectionEnabled(device_handle_, session_index_, param_.filter_ir_distortion);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetIrDistortionCorrectionEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetIrDistortionCorrectionEnabled(device_handle_, session_index_, &ir_correction);
    BOOST_LOG_TRIVIAL(info) << "IR Distortion Correction Filter: " << ir_correction;
  }

  bool compute_correction;
  status = Ps2_SetComputeRealDepthCorrectionEnabled(device_handle_, session_index_, param_.filter_compute_real_depth_correction);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetComputeRealDepthCorrectionEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetComputeRealDepthCorrectionEnabled(device_handle_, session_index_, &compute_correction);
    BOOST_LOG_TRIVIAL(info) << "Compute Real Depth Correction Filter: " << compute_correction;
  }

  bool spatial_filter;
  status = Ps2_SetSpatialFilterEnabled(device_handle_, session_index_, param_.filter_spatial);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetSpatialFilterEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetSpatialFilterEnabled(device_handle_, session_index_, &spatial_filter);
    BOOST_LOG_TRIVIAL(info) << "Spatial Filter: " << spatial_filter;
  }

  bool time_filter;
  status = Ps2_SetTimeFilterEnabled(device_handle_, session_index_, param_.filter_time);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetTimeFilterEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetTimeFilterEnabled(device_handle_, session_index_, &time_filter);
    BOOST_LOG_TRIVIAL(info) << "Time Filter: " << time_filter;
  }

  // Threshold

  if (param_.threshold != 0) {
    status = Ps2_SetThreshold(device_handle_, session_index_, param_.threshold);
    if (status != PsReturnStatus::PsRetOK) {
      BOOST_LOG_TRIVIAL(warning) << "Ps2_SetThreshold failed!" << returnStatus2string(status);
    }
  }

  uint16_t threshold_value;
  status = Ps2_GetThreshold(device_handle_, session_index_, &threshold_value);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_GetThreshold failed!" << returnStatus2string(status);
  } else {
    BOOST_LOG_TRIVIAL(info) << "Threshold value: " << threshold_value;
  }

  // pulse count

  if (param_.pulse_count != 0) {
    status = Ps2_SetPulseCount(device_handle_, session_index_, param_.pulse_count);
    if (status != PsReturnStatus::PsRetOK) {
      BOOST_LOG_TRIVIAL(warning) << "Ps2_SetPulseCount failed!" << returnStatus2string(status);
    }
  }

  uint16_t pulse_count_value;
  status = Ps2_GetPulseCount(device_handle_, session_index_, &pulse_count_value);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_GetPulseCount failed!" << returnStatus2string(status);
  } else {
    BOOST_LOG_TRIVIAL(info) << "Pulse Count value: " << pulse_count_value;
  }

  // GMM Gain (gamma)

  PsGMMGain gmm_options = {param_.gmm_gain, param_.gmm_gain_option};
  if (param_.gmm_gain != 0) {
    status = Ps2_SetGMMGain(device_handle_, session_index_, gmm_options);
    if (status != PsReturnStatus::PsRetOK) {
      BOOST_LOG_TRIVIAL(warning) << "Ps2_SetGMMGain failed!" << returnStatus2string(status);
    }
  }

  uint16_t gmm_options_value;
  status = Ps2_GetGMMGain(device_handle_, session_index_, &gmm_options_value);
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_GetThreshold failed!" << returnStatus2string(status);
  } else {
    BOOST_LOG_TRIVIAL(info) << "GMM Gain value: " << gmm_options_value;
    BOOST_LOG_TRIVIAL(info) << "GMM option: " << param_.gmm_gain_option;
  }

  // Data mode
  BOOST_LOG_TRIVIAL(info) << "-------- Data settings:";

  PsDataMode dataMode = PsDepthAndIR_30;
  // PsDataMode getDataMode;
  status = Ps2_SetDataMode(device_handle_, session_index_, dataMode);

  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetDataMode failed!:" << dataMode2str(dataMode);
  } else {
    status = Ps2_GetDataMode(device_handle_, session_index_, &dataMode);
    if (status != PsReturnStatus::PsRetOK)
      BOOST_LOG_TRIVIAL(warning) << "Ps2_GetDataMode failed!" << returnStatus2string(status);
    else
      BOOST_LOG_TRIVIAL(info) << "Data output mode: " << dataMode2str(dataMode);
  }

  // Synchronize (in time) IR and Depth images within the camera
  bool enableSynchronization = true;
  bool getSyncValue;
  status = Ps2_SetSynchronizeEnabled(device_handle_, session_index_, enableSynchronization);
  
  if (status != PsReturnStatus::PsRetOK) {
    BOOST_LOG_TRIVIAL(warning) << "Ps2_SetSynchronizeEnabled failed!" << returnStatus2string(status);
  } else {
    status = Ps2_GetSynchronizeEnabled(device_handle_, session_index_, &getSyncValue);
    if (status != PsReturnStatus::PsRetOK)
      BOOST_LOG_TRIVIAL(warning) << "Ps2_GetSynchronizeEnabled failed!" << returnStatus2string(status);
    else
      BOOST_LOG_TRIVIAL(info) << "Synchronization between IR and Depth: " << getSyncValue;
  }


  BOOST_LOG_TRIVIAL(info) << "--------";
  




  // TODO(sigurdal): Implement WDR mode?
  // PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
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

void print_PsFrame_info(const PsFrame& frame)
{
  BOOST_LOG_TRIVIAL(trace) << "  frameIndex: " << frame.frameIndex;
  BOOST_LOG_TRIVIAL(trace) << "  frameType: " << frame.frameType;
  BOOST_LOG_TRIVIAL(trace) << "  pixelFormat: " << frame.pixelFormat;
  BOOST_LOG_TRIVIAL(trace) << "  dataLen: " << frame.dataLen;
  BOOST_LOG_TRIVIAL(trace) << "  exposureTime: " << frame.exposureTime;
  BOOST_LOG_TRIVIAL(trace) << "  width: " << frame.width;
  BOOST_LOG_TRIVIAL(trace) << "  height: " << frame.height;
  BOOST_LOG_TRIVIAL(trace) << "  timestamp:";
  BOOST_LOG_TRIVIAL(trace) << "    hour: " << frame.timestamp.tm_hour;
  BOOST_LOG_TRIVIAL(trace) << "    minute: " << frame.timestamp.tm_min;
  BOOST_LOG_TRIVIAL(trace) << "    sec: " << frame.timestamp.tm_sec;
  BOOST_LOG_TRIVIAL(trace) << "    msec: " << frame.timestamp.tm_msec;
}

bool i3ds::VzenseCamera::sample_loop(i3ds_asn1::Timepoint timestamp) {

  // Handle the frames
  PsFrameReady frameReady = {0};
  auto status = Ps2_ReadNextFrame(device_handle_, session_index_, &frameReady);

  if (status != PsRetOK) {
      BOOST_LOG_TRIVIAL(error) << "Error reading frame: " << returnStatus2string(status);
      return false;
  }

/* Sending depth + IR */
  PsFrame depthFrame = {0};
  PsFrame IRFrame = {0};

  if (frameReady.depth == 1 ) {
      
      status = Ps2_GetFrame(device_handle_, session_index_, PsDepthFrame, &depthFrame);
      
      BOOST_LOG_TRIVIAL(trace) << "Depth frame info:";
      print_PsFrame_info(depthFrame);

      if (status != PsRetOK || depthFrame.pFrameData == NULL) {
          BOOST_LOG_TRIVIAL(warning) << "Ps2_GetFrame PsDepthFrame status:" << returnStatus2string(status);
          return true;
      }
  
      if (param_.ir_output && frameReady.ir == 1) {

          status = Ps2_GetFrame(device_handle_, session_index_, PsIRFrame, &IRFrame);
          BOOST_LOG_TRIVIAL(trace) << "IR-frame info:";
          print_PsFrame_info(IRFrame);
          if (status != PsRetOK || IRFrame.pFrameData == NULL) {
            BOOST_LOG_TRIVIAL(warning) << "Ps2_GetFrame PsIRFrame status:" << returnStatus2string(status);
            return true;
          }
      }

      send_sample(depthFrame, IRFrame);
  
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

i3ds_asn1::Timepoint i3ds::VzenseCamera::convert_to_UNIX(const PsTimeStamp& camera_timestamp) {

  // Get current time within i3ds and create a tm struct
  std::time_t t = std::time(nullptr);
  struct tm* i3ds_timestamp;
  i3ds_timestamp = localtime(&t);

  // Replace with camera timestamp values
  i3ds_timestamp->tm_sec = camera_timestamp.tm_sec;
  i3ds_timestamp->tm_min = camera_timestamp.tm_min;
  i3ds_timestamp->tm_hour = camera_timestamp.tm_hour;

  // Fix the timestamp in case the camera's timestamp is at 23h 59m 59s 999ms, and
  // the i3ds timestamp is at 00h 00m 00s 005ms (for example)
  if (camera_timestamp.tm_hour > i3ds_timestamp->tm_hour) {
    i3ds_timestamp->tm_mday -= 1;
  }

  // Convert to UNIX and add milliseconds
  i3ds_asn1::Timepoint unix_timestamp = mktime(i3ds_timestamp)*1000 + camera_timestamp.tm_msec;
  // Convert to microseconds before returning, to make it correspond with other timestamps from i3ds
  return unix_timestamp*1000;
}

void i3ds::VzenseCamera::add_depths_to_depthmap(i3ds::DepthMap& depthMap, const PsFrame& depth_frame)
{
  i3ds_asn1::Timepoint timestamp = convert_to_UNIX(depth_frame.timestamp);

  depthMap.descriptor.attributes.timestamp = timestamp;
  depthMap.descriptor.attributes.validity = i3ds_asn1::SampleValidity_sample_valid;
  depthMap.descriptor.width = depth_frame.width;
  depthMap.descriptor.height = depth_frame.height;

  const uint size = depth_frame.width * depth_frame.height;

  depthMap.depths.resize(size);

  uint16_t* depth_data = reinterpret_cast<uint16_t*>(depth_frame.pFrameData);

  for (size_t i = 0; i < size; i++) {
    if (depth_data[i] == 0xffff) {
      depthMap.depths[i] = -1;
      continue;
    }
    int index = param_.flip_image ? size-1-i : i; // Add values backwards if param_.flip_image is true
    depthMap.depths[index] = static_cast<float>(depth_data[i]) / 1000.0f; // Scale from [mm] to [m]
  }
}

void i3ds::VzenseCamera::add_ir_frame_to_depthmap(i3ds::DepthMap& depthMap, const PsFrame& ir_frame)
{
  i3ds_asn1::Timepoint timestamp = convert_to_UNIX(ir_frame.timestamp);

  depthMap.frame.descriptor.attributes.timestamp = timestamp; 
  depthMap.frame.descriptor.attributes.validity = i3ds_asn1::SampleValidity_sample_valid;
  depthMap.frame.descriptor.image_count = 1;
  depthMap.frame.descriptor.frame_mode  = i3ds_asn1::Frame_mode_t_mode_mono;
  depthMap.frame.descriptor.data_depth  = 16;
  depthMap.frame.descriptor.pixel_size  = 2;
  depthMap.frame.descriptor.region.size_x = ir_frame.width;
  depthMap.frame.descriptor.region.size_y = ir_frame.height;

  size_t image_data_size = ir_frame.dataLen;

  uint16_t* ir_data = reinterpret_cast<uint16_t*>(ir_frame.pFrameData);

  unsigned int n_pixels = image_data_size/2; // Length is halved since we use uint16_t pointer

  for (size_t i = 0; i < n_pixels; i++) {
    ir_data[i] *= 17; // Scaling factor. The highest IR pixel value is 3840.
  }

  if (param_.flip_image) {
    if (flipped_image_ == nullptr) {
      flipped_image_ = std::unique_ptr<uint16_t>(new uint16_t[n_pixels]());
    }
    for (size_t i = 0; i < n_pixels; i++) {
      flipped_image_.get()[n_pixels-1-i] = ir_data[i];
    }
    depthMap.frame.append_image(reinterpret_cast<i3ds_asn1::byte*>(flipped_image_.get()), image_data_size);
  } else {
    depthMap.frame.append_image(ir_frame.pFrameData, image_data_size);
  }

  depthMap.has_frame = true;
}
  
void i3ds::VzenseCamera::send_sample(const PsFrame& depth_frame, const PsFrame& ir_frame) {
  ToFCamera::MeasurementTopic::Data depthMap;
  ToFCamera::MeasurementTopic::Codec::Initialize(depthMap);

  add_depths_to_depthmap(depthMap, depth_frame);
  if (param_.ir_output) {
    add_ir_frame_to_depthmap(depthMap, ir_frame);
  }

  publisher_.Send<ToFCamera::MeasurementTopic>(depthMap);
  update_and_check_batch_count();
}
