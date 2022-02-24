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

#include <thread>

#include "Vzense_api2.h"

#include <i3ds/publisher.hpp>
#include <i3ds/tof_camera_sensor.hpp>

namespace i3ds {

class VzenseCamera : public ToFCamera {
  public:
  struct Parameters {
    // Unique identifier of camera parameter.
    std::string camera_name;
  };

  // Constructor and destructor.
  VzenseCamera(Context::Ptr context, i3ds_asn1::NodeID node, const Parameters &param);
  ~VzenseCamera() {}

  bool is_sampling_supported(i3ds_asn1::SampleCommand sample) {
    // TODO(sigurdm): Not sure what this means
    return false;
  }

  void do_activate();

  void do_deactivate();

  void do_start();

  void do_stop();

  /* UNIMPLEMENTED METHODS from ToFCamera

    // Get the region of interest enabled for ToF-camera.
    virtual bool region_enabled() const {return false;}

    // Get the region of interest for the ToF-camera.
    virtual i3ds_asn1::PlanarRegion region() const {return {0,0,0,0};}

    // Get the min range configuration of the ToF-camera.
    virtual double range_min_depth() const {return 0.0;}

    // Get the max range configuration of the ToF-camera.
    virtual double range_max_depth() const {return 1.0e6;}

    // Attach handlers to the server.
    virtual void Attach(Server& server);
  */
  protected:
  // Constant parameters for Vzense camera.
  const Parameters param_;

  /* UNIMPLEMENTED METHODS:

    // Handler for ToF-camera region of interest command.
    virtual void handle_region(ToFCamera::RegionService::Data& command);

    // Handler for ToF-camera range command.
    virtual void handle_range(ToFCamera::RangeService::Data& command);

    // Handler for camera configuration query.
    virtual void handle_configuration(ToFCamera::ConfigurationService::Data&
    config);
  */
  private:

  bool get_depth_frame();
  void sample_loop();
  void send_sample(const uint16_t *data, uint width, uint height);

  uint32_t slope_;

  std::thread sampler_;
  bool sampler_running_;
  
  uint32_t session_index_;
  PsDeviceHandle device_handle_ = nullptr;

  Publisher publisher_;
};

}  // namespace i3ds

#endif  // __VZENSE_CAMERA_HPP
