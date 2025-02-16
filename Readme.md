# Embedded Stack for ROS 2

Currently the master branch is still in development and is not useable.  
To work on a feature, create a new branch following this naming convention `uniqname_featureName`.  
After a pull request of the feature branch has been merged, delete the feature branch.  


## OdriveS1

### Connecting to ODrive

* First make sure to install a version of Python, either in the terminal(for MacOS users) or through the Anaconda Navigator (required for windows).
* Then type `pip install --upgrade odrive` in the Python terminal
* Connect your device to the ODrive and type `odrivetool` in the terminal.

You should now see that your device has successfully connected to an ODrive. Remember, the ODrives require at least 12V and must be powered separately through a battery to turn the motor.

#### Troubleshooting

If you're on Windows, you may run into an issue where the USB device doesn't open. Open the [Zadig Utility](https://zadig.akeo.ie/) and download the latest version. Make sure to hit 'X' on the advertisement. Then run the `.exe` file and click "Allow." Connect to your ODrive if you have not already, and ensure that you see a `WinUSB` in one box and `ODrive Native Interface` in another box (these are not exact messages). Hit "Install." After a few minutes, the driver will be installed, and typing `odrivetool` in the Python terminal will allow you to successfully interface with the ODrives.

### Connecting ODriveS1 to STM32

**TODO: Update this section to show which pins on the STM32 to connect to**

### ODrive Motor Testing

To work with the motors, follow the encoder and motor setup [here](https://docs.odriverobotics.com/v/latest/getting-started.html#motor-configuration).
Afterwards, calibrate the motors with `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`. Within two seconds, a beep will occur, followed by the calibration. The calibration consists of three different phases. If at any point the calibration fails, the motor will stop working, and the ODrive will update all the error flags.

After a successful calibration sequence, enter `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`. Then type `odrv0.axis0.controller.config.control_mode` to check what mode the ODrive is in. The integer value corresponds to one of four modes, which can be verified [here](https://betadocs.odriverobotics.com/api/odrive.controller.controlmode). For now, set it to `CONTROL_MODE_VELOCITY_CONTROL` by typing `odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`.

To set an input velocity, type `odrv0.axis0.controller.input_vel = x` where x is the value in revolutions per second. The current tested range is (-MAX, 2) ∪ [0, 0] ∪ (2, MAX) where MAX is the maximum revolutions per second allowed. This value can be changed by the user. If at any point the motor stops spinning, it means that the ODrive has errors and that the error flags have been set.

If there are errors at any point, type `dump_errors(odrv0)` to view all errors, and type `odrv0.clear_errors()` to reset all the error flags. Then start over with the calibration sequence.

### ODrive Configuration

The ODrive's configuration can be saved to a JSON file using the command `odrivetool backup-config my_config.json`. It can be saved to an ODrive from the configuration file using `odrivetool restore-config my_config.json` and in `odrivetool`, `odrv0.save_configuration()`. The current configuration is stored under `odrive/config/OdriveS1_current_config`. You must make sure the firmware version for the OdriveS1 is 0.6.7 for the configuration to work. You can update the firmware version by following this [article](https://docs.odriverobotics.com/v/latest/guides/firmware-update.html)


### STM32 Connection to Odrive
Currently, STM32 is connected via UART to the Odrive. You must connect STM32 ground and 3.3V to the ISO ground and ISO VDD on the Odrive S1.

Left wheel Odrive Rx -> STM pin PC6

Left wheel Odrive TX -> STM pin PC7


Right wheel Odrive Rx -> STM pin PA2

Right wheel Odrive TX -> STM pin PA3

### Setup for indicator LED 
1. run this in terminal:
sudo nano /etc/udev/rules.d/99-arduino.rules
2. in the file that opens, add:
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="557363130383514121D2", SYMLINK+="LED_Arduino", MODE="0666"
3. reboot system
Note: 
run "lsusb" to get the idVendor and idProduct
to get serial number, make sure device is connected to ACM0, then run " udevadm info -a -n /dev/ttyACM0 | grep 'ATTRS{serial}' "

