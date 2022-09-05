## Installation

1. Make sure i3ds-framework-cpp is installed.
1. Clone this repository (i3ds-vzense-tof) and make sure that the submodules are included, either by adding `--recursive` when first cloning, or by running `git submodule update --init --recursive` in the checked out repository. This ensures that the Vzense driver is downloaded.
1. Configure and compile
```
mkdir build
cd build
cmake ..
make
```
4. Make sure `src/i3ds_vzense_tof` is created.
5. (Optionally) Install `i3ds_vzense_tof` by running sudo make install

## Operation

To list available cameras, run the program with the -p argument: `i3ds_vzense_tof -p`. This will print the number of available cameras and the information about them. The serial number is what is used as CAMERA-NAME for the -c argument below. The last few characters of this serial number will also be printed on the label on the device.

Start the node using `src/i3ds_vzense_tof`.
A specific camera can be connected to by suppling the `-c CAMERA-NAME` parameter.
By default, the camera will output both depthmaps and IR images, if the IR-images are not needed, they can be disabled with the `--no-ir` flag.
To show information about connected cameras, like serial number, run with the `--print` flag.

To visualize the TOF output, in a new terminal, run: `i3ds_camera_capture --tof`

Then activate the TOF using: `i3ds_configure_tof --activate`

Set the wanted range using `i3ds_configure_tof -D NUM` where NUM is 1 (near), 2 (middle), 3 (far).
The NUM argument gives the max range in meters. The vzense camera supports three ranges:
* Near: 0.35-1.5 m
* Mid: 0.8-4.4 m
* Far: 1.2-6.2 m
 
Start the TOF using `i3ds_configure_tof --start`

Stop the TOF using `i3ds_configure_tof --stop`

Deactivate the TOF using `i3ds_configure_tof --deactivate`

If a different node id is used, specify this using the `-n` parameter for all commands.

## Troubleshooting

* `i3ds/communication.hpp: No such file or directory` - Make sure that i3ds-framework-cpp is installed
* If `i3ds_camera_capture` fails after installation with the message "error while loading shared libraries: libi3ds.so.1.0", try to run `sudo ldconfig` to see if that fixes the problem. If it still persists, try rebooting the computer.
