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
#include <iostream>

#include <ostream>
#include <vzense_pair_client.hpp>


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
    std::cout << "Image data size: " << image_data_size << std::endl;
    std::cout << "First pixel value in image: " << static_cast<int>(image_data[0]) << std::endl;
    std::cout << "Last pixel value in image: " << static_cast<int>(image_data[image_data_size-1]) << std::endl;
}


int main(int argc, char **argv)
{
    // NodeID of VZense ToF cameras have been set to match emulated ToF cameras in i3ds_suite_emulator
    // Set up by running
    //   $ i3ds_suite_emulator
    // and
    //   $ i3ds_suite_emulator --base 110
    // as two different processes.
    // Note that the data from both sensors can look the same if the RNG has the same seed. A solution then
    // is to restart one of the emulator-suites after som messages has been sent to reinitialize the RNG.
    i3ds_asn1::NodeID left_camera_node_id = 15;
    i3ds_asn1::NodeID right_camera_node_id = 115;

    // Create I3DS objects
    i3ds::Context::Ptr context(i3ds::Context::Create());
    VZensePairClient client(context, left_camera_node_id, right_camera_node_id, 0);

    // Activate sensors so we can configure them
    client.left_client.load_status();
    if (client.left_client.is_inactive()) {
        client.left_client.Activate();
    } else if (client.left_client.is_operational()) {
        client.left_client.Stop();
    }
    client.right_client.load_status();
    if (client.right_client.is_inactive()) {
        client.right_client.Activate();
    } else if (client.right_client.is_operational()) {
        client.right_client.Stop();
    }

    // Configure sensors
    i3ds_asn1::SamplePeriod period = 100000;
    client.left_client.set_sampling(period);
    client.right_client.set_sampling(period);

    running = true;
    while (running) {

        std::cout << "Press Enter to trigger a message from each camera. Ctrl-C to exit.\n";
        std::cin.get();
        // Wait a second
        sleep(1);

        // Trigger a measurement
        client.trigger();

        // Wait for new measurement
        while(!client.valid_measurements()) {
            std::cout << "waiting for measurement\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Do something with the measurements
        handle_measurement(client.fetch_left_measurement());
        handle_measurement(client.fetch_right_measurement());
    }

    return 0;
}