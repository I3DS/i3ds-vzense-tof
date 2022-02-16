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

#include <i3ds/tof_camera_sensor.hpp>

#include "Vzense_api2.h"

namespace i3ds {

class VzenseCamera : public ToFCamera {
public:
  struct Parameters
  {
    // Unique identifier of camera parameter.
    std::string camera_name;
  };

  // Constructor and destructor.
  VzenseCamera(i3ds_asn1::NodeID node, const Parameters &param);
  ~VzenseCamera() { }

  bool is_sampling_supported(i3ds_asn1::SampleCommand sample) {
    //TODO(sigurdm): implement
    return false;
  }

  void do_activate();

  void do_deactivate() {
    //TODO(sigurdm): implement
  }

  void do_start() {
    //TODO(sigurdm): implement
  }

  void do_stop() {
    //TODO(sigurdm): implement
  }

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

  PsDeviceHandle deviceHandle_ = nullptr;
/* UNIMPLEMENTED METHODS:

  // Handler for ToF-camera region of interest command.
  virtual void handle_region(ToFCamera::RegionService::Data& command);

  // Handler for ToF-camera range command.
  virtual void handle_range(ToFCamera::RangeService::Data& command);

  // Handler for camera configuration query.
  virtual void handle_configuration(ToFCamera::ConfigurationService::Data& config);
*/
};

} // namespace i3ds

#endif  // __VZENSE_CAMERA_HPP
