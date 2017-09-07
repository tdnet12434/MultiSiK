SiK - Multipoint Branch
=====
Firmware for SiLabs Si1000 - Si102x/3x ISM radios

SiK is a collection of firmware and tools for radios based on the cheap, versatile SiLabs Si1000 SoC.

## Branch Build Status
[![Build Status](http://jenkins.hovo.id.au/buildStatus/icon?job=SiK_Multipoint)](http://jenkins.hovo.id.au/job/SiK_Multipoint/)

## Documentation

For user documentation please see this site:

http://planner.ardupilot.com/wiki/other-project-and-common-topics/common-optional-hardware/common-telemetry-landingpage/common-3dr-radio-version-2/

Addition configuration guide can also be found here:

http://copter.ardupilot.com/wiki/common-optional-hardware/common-telemetry-landingpage/common-3dr-radio-advanced-configuration-and-technical-information/

SiK is a collection of firmware and tools for radios based on the cheap, versatile SiLabs Si1000 SoC.

Currently, it supports the following boards:

 - HopeRF HM-TRP
 - HopeRF RF50-DEMO
 - RFDesign RFD900a
 - RFDesign RFD900u
 - RFDesign RFD900p

Adding support for additional boards should not be difficult.

Currently the firmware components include:

 - A bootloader with support for firmware upgrades over the serial interface.
 - Radio firmware with support for parsing AT commands, storing parameters and FHSS/TDM functionality

See the user documentation above for a list of current firmware features

## Multipoint Support - Differences from standard image
### New AT Variables

###### S15: NODEID
MUST be a unique ID on the network, otherwise cross talk will occur
Base ID is defined by setting to 0, this is the node that keeps the network in sync with all other nodes.
When setting the the NODEID to the base (ie 0) it will always have link to it's self thus link will never be lost.

###### S16: NODEDESTINATION
This is where all the serial data recived on this node should be sent to.
For example, to send all data to the base node only set this to 0.
DEFAULT is 65535 which is broadcast to all nodes.

###### S17: SYNCANY
Sync Any should be used sparingly, this allows any node to sync from any node in the network not just the base.
By allowing many nodes to sync from each other they could drift away from the main sync channel,
thus efectivly becoming a diffrent NETID.

###### S18: NODECOUNT
This is the number of nodes in the nework, so if you have 3 nodes (Id's 0, 1, 2) you have a network of 3.
In the next revision of the software this could disapear and become auto detect.
The easiest way to determin the correct node count for the network is - max(NODEID)+1

### Diffrent RT Syntax

All RT's are sent to every node by default, to send them to one node only folow the command with a comma and node id.
Example getting the parameter list from base node would be 'RTI5,0'

## What You Will Need

 - A Mac OS X or Linux system for building.  Mac users will need the Developer Tools (Xcode) installed.
 - At least two Si1000 - Si102x/3x - based radio devices (just one radio by itself is not very useful).
 - A [SiLabs USB debug adapter](http://www.silabs.com/products/mcu/Pages/USBDebug.aspx).
 - [SDCC](http://sdcc.sourceforge.net/), version 3.3.1 #8719 or later.
 - [EC2Tools](http://github.com/tridge/ec2)
 - [Mono](http://www.mono-project.com/) to build and run the GUI firmware updater.
 - Python to run the command-line firmware updater, with packages pexpect and argparse

Note that at this time, building on Windows systems is not supported.  If someone wants to contribute and maintain the necessary pieces that would be wonderful.

## Building Things

Type `make install` in the Firmware directory.  If all is well, this will produce a folder called `dst` containing bootloader and firmware images.

If you want to fine-tune the build process, `make help` will give you more details.

Building the SiK firmware generates bootloaders and firmware for each of the supported boards. Many boards are available tuned to specific frequencies, but have no way for software on the Si1000 to detect which frequency the board is configured for. In this case, the build will produce different versions of the bootloader for each board. It's important to select the correct bootloader version for your board if this is the case.

## Flashing and Uploading

The SiLabs debug adapter can be used to flash both the bootloader and the firmware. Alternatively, once the bootloader has been flashed the updater application can be used to update the firmware (it's faster than flashing, too).

The `Firmware/tools/ec2upload` script can be used to flash either a bootloader or firmware to an attached board with the SiLabs USB debug adapter.  Further details on the connections required to flash a specific board should be found in the `Firmware/include/board_*.h` header for the board in question.

To use the updater application, open the `SiKUploader/SikUploader.sln` Mono solution file, build and run the application. Select the serial port connected to your radio and the appropriate firmware `.hex` file for the firmware you wish to uploader.  You will need to get the board into the bootloader; how you do this varies from board to board, but it will normally involve either holding down a button or pulling a pin high or low when the board is reset or powered on. 

For the supported boards:

 - HM-TRP: hold the CONFIG pin low when applying power to the board.
 - RF50-DEMO: hold the ENTER button down and press RST.
 - RFD900x: hold the BOOT/CTS pin low when applying power to the board.

The uploader application contains a bidirectional serial console that can be used for interacting with the radio firmware.

As an alternative to the Mono uploader, there is a Python-based command-line upload tool in `Firmware/tools/uploader.py`.

## Supporting New Boards

Take a look at `Firmware/include/board_*.h` for the details of what board support entails.  It will help to have a schematic for your board, and in the worst case, you may need to experiment a little to determine a suitable value for EZRADIOPRO_OSC_CAP_VALUE.  To set the frequency codes for your board, edit the corresponding `Firmware/include/rules_*.mk` file.

## Resources

SiLabs have an extensive collection of documentation, application notes and sample code available online.

Start at the [Si1000 product page](http://www.silabs.com/products/wireless/wirelessmcu/Pages/Si1000.aspx) or [Si102x/3x product page](http://www.silabs.com/products/wireless/wirelessmcu/Pages/Si102x-3x.aspx)

## Reporting Problems

Please use the GitHub issues link at the top of the [project page](http://github.com/tridge/SiK) to report any problems with, or to make suggestions about SiK.  I encourage you to fork the project and make whatever use you may of it.

## AES Branch
With the use of magical kittens there is a sub folder inside the radio project 'AES'.   
The magic comes from the ability to selectively compile sub folders depending on the cpu compiling to.   
Thus not having to worry about messy ifdefs and the like for files in the AES folder when compiling with a unsupported CPU

## What does SiK mean?

It should really be Sik, since 'K' is the SI abbreviation for Kelvin, and what I meant was 'k', i.e. 1000.  Someday I might change it.
