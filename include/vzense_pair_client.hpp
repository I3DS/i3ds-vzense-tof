///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <i3ds/communication.hpp>
#include <i3ds/depthmap.hpp>
#include <i3ds/measurement_trigger.hpp>
#include <i3ds/tof_camera_client.hpp>
#include <i3ds/tof_camera_sensor.hpp>
#include <i3ds_asn1/Common.hpp>

class VZensePairClient
{
public:
    VZensePairClient(i3ds::Context::Ptr context, i3ds_asn1::NodeID left_node, i3ds_asn1::NodeID right_node, unsigned long us_trigger_interval=0) :
    left_client(context, left_node),
    right_client(context, right_node),
    left_trigger_(context, left_node, true),
    right_trigger_(context, right_node, true),
    us_trigger_interval_(us_trigger_interval)
    {}

    // Trigger aquisition from both cameras
    inline void trigger()
    {
        left_trigger_.trigger();
        std::this_thread::sleep_for(std::chrono::milliseconds(us_trigger_interval_));
        right_trigger_.trigger();
    }

    // Returns a reference to the latest measurement of the left camera.
    // Calling trigger() will overwrite this.
    // Throws std::logic_error if no measurement is available.
    inline i3ds::DepthMap& fetch_left_measurement()
    {
        return left_trigger_.fetch();
    }

    // Returns a reference to the latest measurement of the right camera.
    // Calling trigger() will overwrite this.
    // Throws std::logic_error if no measurement is available.
    inline i3ds::DepthMap& fetch_right_measurement()
    {
        return right_trigger_.fetch();
    }

    inline bool valid_measurements()
    {
        return left_trigger_.valid_measurement() && right_trigger_.valid_measurement();
    }

    i3ds::ToFCameraClient left_client;
    i3ds::ToFCameraClient right_client;

private:

    i3ds::Context::Ptr context_;

    i3ds::MeasurementTrigger<i3ds::ToFCamera::MeasurementTopic> left_trigger_;
    i3ds::MeasurementTrigger<i3ds::ToFCamera::MeasurementTopic> right_trigger_;

    unsigned long us_trigger_interval_;
};