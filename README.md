# Xsens MTi driver for ROS

# Changelog:
This package is almost completely taken from the ROS example in the MT SDK.
Changes done with respect to the orginal source:
 - Adapted CMakelists.txt to also install TARGETS
 - Adapted CMakelists.txt to run a PRE-BUILD command that builds libraries needed for the executable
 - Removed all trailing whitespaces
 - Added DiagnosedPublisher to the ImuPublisher

# Documentation:
  For this you need the full MT SDK which can be downloaded from: https://www.xsens.com/software-downloads
  You can find the full documentation in "<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html" under "ROS MTi driver" section.

# Prerequisites:
  - ROS Kinetic, Melodic or Noetic
  - C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
  - C++11

# Building:
  CMakelists.txt has been adapted from official xsens package.
  It now automatically builds the library via a PRE_BUILD call

  Therefore a simple catkin build --this should suffice

# Running:
  - Configure your MTi device to output desired data (e.g. for display example - orientation output) via MT Manager

   - Launch the Xsens MTi driver from your catkin workspace:


    $ roslaunch xsens_mti_driver xsens_mti_node.launch

  or with rviz visualization:

    $ roslaunch xsens_mti_driver display.launch


Notes:
    - ROS timestamps
        The data messages from devices are time stamped on arrival in the ROS driver.
        When collecting data at higher rates, eg 100 Hz, the times between reads can differ from the configured output rate in the device.
        This is caused by possible buffering in the USB/FTDI driver.

        For instance:
        10 us, 10 us, 10 us, 45 ms, 10 us, 10 us, 10 us, 39 ms, 10 us, etc.
        instead of 
        10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, etc.

        Work-around: for accurate and stable time stamp information, users can make use of the sensor reported time stamp (/imu/time_ref) instead.

-[ Troubleshooting ]------------------------------------------------------------

    - The Mti1 (Motion Tracker Development Board) is not recognized.

        Support for the Development Board is present in recent kernels. (Since June 12, 2015).
        If your kernel does not support the Board, you can add this manually

        $ sudo /sbin/modprobe ftdi_sio
        $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


    - The device is recognized, but I cannot ever access the device -

        Make sure you are in the correct group (often dialout or uucp) in order to
        access the device. You can test this with

            $ ls -l /dev/ttyUSB0
            crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
            $ groups
            dialout audio video usb users plugdev

        If you aren't in the correct group, you can fix this in two ways.

        1. Add yourself to the correct group
            You can add yourself to it by using your distributions user management
            tool, or call

                $ sudo usermod -G dialout -a $USER

            Be sure to replace dialout with the actual group name if it is
            different. After adding yourself to the group, either relogin to your
            user, or call

                $ newgrp dialout

            to add the current terminal session to the group.

        2. Use udev rules
            Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

                SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

            Change $GROUP into your desired group (e.g. adm, plugdev, or usb).


    - The device is inaccessible for a while after plugging it in -

        When having problems with the device being busy the first 20 seconds after
        plugin, purge the modemmanager application.

    - RViz doesn't show an MTi model.

        It is a known issue with urdfdom in ROS Melodic. A workaround is to unset/modify the LC_NUMERIC environment variable:

        $ LC_NUMERIC="en_US.UTF-8"
