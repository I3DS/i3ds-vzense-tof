///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __VZENSE_CAMERA_HPP
#define __VZENSE_CAMERA_HPP

#include <memory>
#include <thread>

#include "DCAM710/Vzense_types_710.h"
#include "Vzense_api2.h"

#include <i3ds/publisher.hpp>
#include <i3ds/tof_camera_sensor.hpp>
#include <i3ds/periodic.hpp>

namespace i3ds {

class VzenseCamera : public ToFCamera {
  public:
  struct Parameters {
    // Unique identifier of camera parameter.
    std::string camera_name;

    // Output IR image along with depthmap
    bool ir_output;

    // Flip image 180 degrees
    bool flip_image;

    // filters
    bool filter_depth_distortion;
    bool filter_ir_distortion;
    bool filter_compute_real_depth_correction;
    bool filter_spatial;
    bool filter_time;

    // threshold setting
    uint16_t threshold;
    uint16_t pulse_count;
    uint16_t gmm_gain;
    uint8_t gmm_gain_option;
  };

  // Constructor and destructor.
  VzenseCamera(Context::Ptr context, i3ds_asn1::NodeID node, const Parameters &param);
  ~VzenseCamera() {}

  bool is_sampling_supported(i3ds_asn1::SampleCommand sample) {
    // Frame rate must be between 1 Hz and 30 Hz
    return 33333 <= sample.period && sample.period <= 1000000;
  }

  void do_activate();

  void do_deactivate();

  void do_start();

  void do_stop();

  // Get the min range configuration of the ToF-camera.
  double range_min_depth() const;

  // Get the max range configuration of the ToF-camera.
  double range_max_depth() const;

  protected:
  // Constant parameters for Vzense camera.
  const Parameters param_;

  // Handler for ToF-camera range command.
  virtual void handle_range(ToFCamera::RangeService::Data& command);
  
  private:

  i3ds_asn1::Timepoint convert_to_UNIX(const PsTimeStamp& camera_timestamp);
  void add_depths_to_depthmap(i3ds::DepthMap& depthMap, const PsFrame& depth_frame);
  void add_ir_frame_to_depthmap(i3ds::DepthMap& depthMap, const PsFrame& ir_frame);
  bool sample_loop(i3ds_asn1::Timepoint timestamp);
  void send_sample(const PsFrame& depth_data, const PsFrame& ir_data);

  Sampler sampler_;
  
  uint32_t session_index_;
  PsDeviceHandle device_handle_ = nullptr;

  PsDepthRange wanted_range_ = PsFarRange;

  Publisher publisher_;

  // Memory space to copy flipped image if needed
  std::unique_ptr<uint16_t> flipped_image_;
};

}  // namespace i3ds

#endif  // __VZENSE_CAMERA_HPP
