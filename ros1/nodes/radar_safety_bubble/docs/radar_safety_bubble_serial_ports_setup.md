mmWave Driver Serial Ports Setup for Radar Safety Bubble Demo
=============================================================

## Serial Ports

For each mmWave EVM that you connect to the SK board 2 serial ports are assigned (ex: `/dev/ttyUSB0` and `/dev/ttyUSB1` or `/dev/ttyACM0` and `/dev/ttyACM1`). One port (usually the first one numerically) will be used for configuring the mmWave device (command_port), the other will be used for data transfer (data_port). Before you will be able to run the demo, you must find the serial port names associated with the mmWave devices that you wish to use.

### Single Sensor Setup

Users with a single sensor can find the command and data port names with the following:

1. Run `ls /dev` without the mmWave EVM connected to the SK board.
2. Connect the mmWave EVM to the SK board with a micro USB cable.
3. Again run `ls /dev`. Any new entries will be the serial ports associated with your mmWave EVM.
4. Open `/opt/robot/radar-safety-bubble/launch/single/ti_mmwave_sensor.launch` and ensure that the values for the `command_port` and `data_port` parameters match the serial ports for your EVM.
5. Save and close the file.

### Four Sensor Setup

Users with multiple sensors may wish to create symbolic links. If symbolic links are not used then the EVMs must be connected in a specific order each time the demo is run, which can become quite tedious and confusing. Follow the steps below to set up symbolic links:

**Note**: The following steps work with standalone EVMs only and will not work with the MMWAVE-ICBOOST carrier board.

1. On the TDA4 host linux, create a file named `99-usb-serial.rules` in the directory `/etc/udev/rules.d`
2. In the new file, type or paste the following:
    ```
    SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", SYMLINK+="mmWave_%s{serial}_%E{ID_USB_INTERFACE_NUM}"
    ```
3. Close and save the file.
4. The symlink port names for each sensor can be found by plugging in the sensor, then typing: `ls /dev | grep mmWave`
    Ex: `mmWave_00CE0FCA_00`, typically the command port and `mmWave_00CE0FCA01`, typically the data port.
5. Open `/opt/robot/radar-safety-bubble/launch/quad/bubble_sensor_forward_facing.launch`
6. Edit the `command_port` and `data_port` parameters to match the serial port names for the forward-facing EVM.
7. Save and close the file
8. Repeat steps 4-7 plugging in each EVM and changing the `command_port` and `data_port` fields in the corresponding launch files: `bubble_sensor_right_facing.launch`, `bubble_sensor_left_facing.launch`, and `bubble_sensor_backward_facing.launch` (all located under `/opt/robot/radar-safety-bubble/launch/quad/`).