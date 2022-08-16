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
#include <i3ds/tof_camera_client.hpp>
#include <i3ds/subscriber.hpp>
#include <i3ds/depthmap.hpp>
#include <i3ds/tof_camera_sensor.hpp>
#include <ostream>
#include <unistd.h>
#include <iostream>


// Flag and signal handler for graceful shutdown
std::atomic<bool> running;

void signal_handler(int)
{
  running = false;
}


// Handler function for ToF camera measurements
void handle_measurement(i3ds::DepthMap& dm)
{
    std::cout << "Received a measurement with timestamp: " << dm.descriptor.attributes.timestamp << std::endl;

    // Accessing depth information
    std::cout << "The size of the depthmap is: " << dm.descriptor.height 
              << " by " << dm.descriptor.width << " pixels" << std::endl;
    std::cout << "Depth value in upper right corner is: " << dm.depths.at(0) << std::endl;
    int dm_center_index = (dm.descriptor.width*dm.descriptor.height/2) + (dm.descriptor.width/2);
    std::cout << "Depth value in center is: " << dm.depths[dm_center_index] << std::endl;

    
    // Accessing image information with dm.frame. We can get info about the image from dm.frame.descriptor
    // Note that this is not the same object as dm.descriptor which is for the depth map.
    std::cout << "Image has " << dm.frame.descriptor.pixel_size << " bytes per pixel" << std::endl;

    // See i3ds_asn1/Frame.hpp for description of this enum
    std::cout << "The image format type is: " << dm.frame.descriptor.frame_mode << std::endl;

    // Image data is access with dm.frame.image_data(0). Since there is only one image,
    // the argument to image_data should always be 0.
    const unsigned char* image_data = dm.frame.image_data(0);
    size_t image_data_size = dm.frame.image_size(0);
    std::cout << "Last pixel value in image: " << image_data[image_data_size-1] << std::endl;
}


int main(int argc, char **argv)
{
    // NodeID to send commands to
    i3ds_asn1::NodeID command_node = 10;

    // NodeID to receive measurements from
    i3ds_asn1::NodeID subscribe_node = 11;
    
    // Create I3DS objects
    i3ds::Context::Ptr context(i3ds::Context::Create());
    i3ds::ToFCameraClient client(context, command_node);
    i3ds::Subscriber subscriber(context);

    // Set up subscriber
    subscriber.Attach<i3ds::ToFCamera::MeasurementTopic>(subscribe_node, handle_measurement);
    subscriber.Start();

    // Activate sensor so we can configure it
    client.Activate();

    // Configure sensor to only send one measurement at a time
    i3ds_asn1::SamplePeriod period = 100000;
    i3ds_asn1::BatchSize batch_size = 1;
    i3ds_asn1::BatchCount batch_count = 1;
    client.set_sampling(period, batch_size, batch_count);

    running = true;
    while (running) {
        // Wait a second
        sleep(1);

        // Send a measurement. Expect to lose the first one or two measurements.
        client.Start();
    }

    subscriber.Stop();

    return 0;
}