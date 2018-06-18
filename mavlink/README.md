# MAVLink

This directory contains various programs/scripts to communicate with AirSim using MavLink.

## MAVLink messages AirSim listens to

AirSim handling MAVLink messages can be seen in the [processMavMessages](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/controllers/MavLinkDroneController.hpp#L565) and the messages it listens to can be summarized in the following tables:

### [HEARTBEAT](http://mavlink.org/messages/common#HEARTBEAT): The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).

| Field Name      | Type                    | Description                                                                                                       | 
|-----------------|-------------------------|-------------------------------------------------------------------------------------------------------------------| 
| type            | uint8_t                 | Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)                           | 
| autopilot       | uint8_t                 | Autopilot type / class. defined in MAV_AUTOPILOT ENUM                                                             | 
| base_mode       | uint8_t                 | System mode bitfield, as defined by MAV_MODE_FLAG enum                                                            | 
| custom_mode     | uint32_t                | A bitfield for use for autopilot-specific flags                                                                   | 
| system_status   | uint8_t                 | System status flag, as defined by MAV_STATE enum                                                                  | 
| mavlink_version | uint8_t_mavlink_version | MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version | 

### [STATUSTEXT](https://mavlink.io/en/messages/common.html#STATUSTEXT): Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).

| Field Name | Type     | Description                                                                                                | 
|------------|----------|------------------------------------------------------------------------------------------------------------| 
| severity   | uint8_t  | Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY. (Enum:MAV_SEVERITY ) | 
| text       | char[50] | Status text message, without null termination character                                                    | 

### [COMMAND_LONG](http://mavlink.org/messages/common#COMMAND_LONG)

| Field Name       | Type     | Description                                                                                      | 
|------------------|----------|--------------------------------------------------------------------------------------------------| 
| target_system    | uint8_t  | System which should execute the command                                                          | 
| target_component | uint8_t  | Component which should execute the command, 0 for all components                                 | 
| command          | uint16_t | Command ID, as defined by MAV_CMD enum.                                                          | 
| confirmation     | uint8_t  | 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) | 
| param1           | float    | Parameter 1, as defined by MAV_CMD enum.                                                         | 
| param2           | float    | Parameter 2, as defined by MAV_CMD enum.                                                         | 
| param3           | float    | Parameter 3, as defined by MAV_CMD enum.                                                         | 
| param4           | float    | Parameter 4, as defined by MAV_CMD enum.                                                         | 
| param5           | float    | Parameter 5, as defined by MAV_CMD enum.                                                         | 
| param6           | float    | Parameter 6, as defined by MAV_CMD enum.                                                         | 
| param7           | float    | Parameter 7, as defined by MAV_CMD enum.                                                         | 

### [HIL_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_CONTROLS): Sent from autopilot to simulation. Hardware in the loop control outputs

| Field Name     | Type     | Description                                                                             | 
|----------------|----------|-----------------------------------------------------------------------------------------| 
| time_usec      | uint64_t | Timestamp (microseconds since UNIX epoch or microseconds since system boot) (Units: us) | 
| roll_ailerons  | float    | Control output -1 .. 1                                                                  | 
| pitch_elevator | float    | Control output -1 .. 1                                                                  | 
| yaw_rudder     | float    | Control output -1 .. 1                                                                  | 
| throttle       | float    | Throttle 0 .. 1                                                                         | 
| aux1           | float    | Aux 1, -1 .. 1                                                                          | 
| aux2           | float    | Aux 2, -1 .. 1                                                                          | 
| aux3           | float    | Aux 3, -1 .. 1                                                                          | 
| aux4           | float    | Aux 4, -1 .. 1                                                                          | 
| mode           | uint8_t  | System mode (MAV_MODE) (Enum:MAV_MODE )                                                 | 
| nav_mode       | uint8_t  | Navigation mode (MAV_NAV_MODE)                                                          | 

### [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS): Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)

| Field Name            | Type      | Description                                                                             | 
|-----------------------|-----------|-----------------------------------------------------------------------------------------| 
| time_usec             | uint64_t  | Timestamp (microseconds since UNIX epoch or microseconds since system boot) (Units: us) | 
| controls              | float[16] | Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.          | 
| mode                  | uint8_t   | System mode (MAV_MODE), includes arming state. (Enum:MAV_MODE )                         | 
| flags                 | uint64_t  | Flags as bitfield, reserved for future use.                                             | 

## MAVLink messages AirSim is sending out

AirSim sends out MAVLink messages by calling its [update](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/controllers/MavLinkDroneController.hpp#L776) method and a summary of those messages can be found in the following tables:

### [HIL_SENSOR](https://mavlink.io/en/messages/common.html#HIL_SENSOR): The IMU readings in SI units in NED body frame

| Field Name     | Type     | Description                                                                                                                                                              | 
|----------------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------| 
| time_usec      | uint64_t | Timestamp (microseconds, synced to UNIX time or since system boot) (Units: us)                                                                                           | 
| xacc           | float    | X acceleration (m/s^2) (Units: m/s/s)                                                                                                                                    | 
| yacc           | float    | Y acceleration (m/s^2) (Units: m/s/s)                                                                                                                                    | 
| zacc           | float    | Z acceleration (m/s^2) (Units: m/s/s)                                                                                                                                    | 
| xgyro          | float    | Angular speed around X axis in body frame (rad / sec) (Units: rad/s)                                                                                                     | 
| ygyro          | float    | Angular speed around Y axis in body frame (rad / sec) (Units: rad/s)                                                                                                     | 
| zgyro          | float    | Angular speed around Z axis in body frame (rad / sec) (Units: rad/s)                                                                                                     | 
| xmag           | float    | X Magnetic field (Gauss) (Units: gauss)                                                                                                                                  | 
| ymag           | float    | Y Magnetic field (Gauss) (Units: gauss)                                                                                                                                  | 
| zmag           | float    | Z Magnetic field (Gauss) (Units: gauss)                                                                                                                                  | 
| abs_pressure   | float    | Absolute pressure in millibar (Units: mbar)                                                                                                                              | 
| diff_pressure  | float    | Differential pressure (airspeed) in millibar (Units: mbar)                                                                                                               | 
| pressure_alt   | float    | Altitude calculated from pressure                                                                                                                                        | 
| temperature    | float    | Temperature in degrees celsius (Units: degC)                                                                                                                             | 
| fields_updated | uint32_t | Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. | 

### [DISTANCE_SENSOR](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR)

| Field Name       | Type     | Description                                                                                                                                                                                                                                                                                         | 
|------------------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------| 
| time_boot_ms     | uint32_t | Time since system boot (Units: ms)                                                                                                                                                                                                                                                                  | 
| min_distance     | uint16_t | Minimum distance the sensor can measure in centimeters (Units: cm)                                                                                                                                                                                                                                  | 
| max_distance     | uint16_t | Maximum distance the sensor can measure in centimeters (Units: cm)                                                                                                                                                                                                                                  | 
| current_distance | uint16_t | Current distance reading (Units: cm)                                                                                                                                                                                                                                                                | 
| type             | uint8_t  | Type from MAV_DISTANCE_SENSOR enum. (Enum:MAV_DISTANCE_SENSOR )                                                                                                                                                                                                                                     | 
| id               | uint8_t  | Onboard ID of the sensor                                                                                                                                                                                                                                                                            | 
| orientation      | uint8_t  | Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270 (Enum:MAV_SENSOR_ORIENTATION ) | 
| covariance       | uint8_t  | Measurement covariance in centimeters, 0 for unknown / invalid readings (Units: cm)                                                                                                                                                                                                                 | 

### [HIL_GPS](https://mavlink.io/en/messages/common.html#HIL_GPS): The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).

| Field Name         | Type     | Description                                                                                                                                               | 
|--------------------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------| 
| time_usec          | uint64_t | Timestamp (microseconds since UNIX epoch or microseconds since system boot) (Units: us)                                                                   | 
| fix_type           | uint8_t  | 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. | 
| lat                | int32_t  | Latitude (WGS84), in degrees * 1E7 (Units: degE7)                                                                                                         | 
| lon                | int32_t  | Longitude (WGS84), in degrees * 1E7 (Units: degE7)                                                                                                        | 
| alt                | int32_t  | Altitude (AMSL, not WGS84), in meters * 1000 (positive for up) (Units: mm)                                                                                | 
| eph                | uint16_t | GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535                                                                         | 
| epv                | uint16_t | GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535                                                                           | 
| vel                | uint16_t | GPS ground speed in cm/s. If unknown, set to: 65535 (Units: cm/s)                                                                                         | 
| vn                 | int16_t  | GPS velocity in cm/s in NORTH direction in earth-fixed NED frame (Units: cm/s)                                                                            | 
| ve                 | int16_t  | GPS velocity in cm/s in EAST direction in earth-fixed NED frame (Units: cm/s)                                                                             | 
| vd                 | int16_t  | GPS velocity in cm/s in DOWN direction in earth-fixed NED frame (Units: cm/s)                                                                             | 
| cog                | uint16_t | Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535 (Units: cdeg)                | 
| satellites_visible | uint8_t  | Number of satellites visible. If unknown, set to 255                                                                                                      | 
