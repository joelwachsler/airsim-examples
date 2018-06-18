# Official API using RPC

AirSim can be controlled and communicated with using the official API described [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md) with [MsgPack-RPC](https://msgpack.org/). This API is ideal if the user wants to get going a quick as possible with the least amount of effort. This API comes with a cost of not being able to gain full control over the motors and sensors and if that's of concern go and have a look at the lower level API accessible with MAVLink [here](/code/mavlink).

The following methods were found by observing [MultirotorRPCLibServer](https://github.com/Microsoft/AirSim/blob/master/AirLib/src/vehicles/multirotor/api/MultirotorRpcLibServer.cpp), [MultirotorRpcLibClient](https://github.com/Microsoft/AirSim/blob/master/AirLib/src/vehicles/multirotor/api/MultirotorRpcLibClient.cpp), [DroneControllerBase](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/controllers/DroneControllerBase.hpp) and [DroneShell](https://github.com/Microsoft/AirSim/blob/master/DroneShell/src/main.cpp).

## Available methods

### enableApiControl

Tells the controller to switch from human operated mode to computer operated mode.

NOTE: all the movement commands, except hover, require you first call enableApiControl(true). If that succeeds it means your application now has control over the drone and the user cannot control the drone unless they flip a switch on their controller taking control back from your program. If the user takes control back and you call one of these methods, an exception will be thrown telling your program that it no longer has control. If you call enableApiControl(false) the drone will hover waiting for user RC input.

### armDisarm

The drone must be armed before it will fly. Set arm to true to arm the drone. On some drones arming may cause the motors to spin on low throttle, this is normal. Set arm to false to disarm the drone. This will disable the motors, so don't do that unless the drone is on the ground! Arming the drone also sets the "home position" This home position is local position x=0,y=0,z=0. You can also query what GPS location that is via getHomeGeoPoint.

| Parameter | Type | Description         | 
|-----------|------|---------------------| 
| arm       | bool | Arm = 1, Disarm = 0 | 

### setSimulationMode

### takeoff

When armed you can tell the drone to takeoff. This will fly to a preset altitude (like 2.5 meters) above the home position. Once the drone is safely in the air you can use other commands to fly from there. If the drone is already flying takeoff will be ignored. Pass non-zer max_wait_seconds if you want the method to also wait until the takeoff altitude is achieved.

| Parameter        | Type  | Description                                | 
|------------------|-------|--------------------------------------------| 
| max_wait_seconds | float | specify time to wait after issuing command | 

### land

At any point this command will disable offboard control and land the drone at the current GPS location. How quickly the drone descends is up to the drone. Some models will descend slowly if they have no lidar telling them how far it is to the ground, while others that can see the ground will descend more quickly until they get near the ground. None of that behavior is defined in this API because it is depends on what kind of hardware the drone has onboard. Pass non-zer max_wait_seconds if you want the method to also wait until the drone reports it has landed, the timeout here is a bit tricky, depends on how high you are and what the drone's configured descent velocity is. If you don't want to wait pass zero. You can also periodically check getLandedState to see if it has landed.

| Parameter        | Type  | Description              | 
|------------------|-------|--------------------------| 
| max_wait_seconds | float | NO DESCRIPTION AVAILABLE | 

### goHome

This command is a safety measure, at any point this command will cancel offboard control and send the drone back to the launch point (or home position). Most drones are also configured to climb to a safe altitude before doing that so they don't run into a tree on the way home

### moveByAngleZ

Move the drone by controlling the angles (or attitude) of the drone, if you set pitch, roll to zero and z to the current z value then it is equivalent to a hover command. A little bit of pitch can make the drone move forwards, a little bit of roll can make it move sideways. The yaw control can make the drone spin around on the spot. The duration says how long you want to apply these settings before reverting to a hover command. So you can say "fly forwards slowly for 1 second" using moveByAngleZ(0.1, 0, z, yaw, 1, ...). The cancelable_action can be used to canel all actions. In fact, every time you call another move* method you will automatically cancel any previous action that is happening.

| Parameter | Type  | Description                             | 
|-----------|-------|-----------------------------------------| 
| pitch     | float | Pitch angle in degrees                  | 
| roll      | float | Roll angle in degrees                   | 
| z         | float | z position in meters                    | 
| duration  | float | The duration of this command in seconds | 

### moveByAngleThrottle

Move by providing angles and throttles just like in RC.

| Parameter | Type  | Description                             | 
|-----------|-------|-----------------------------------------| 
| pitch     | float | Pitch angle in degrees                  | 
| roll      | float | Roll angle in degrees                   | 
| throttle  | float | Target yaw rate in degrees/sec          | 
| yaw_rate  | float | z position in meters                    | 
| duration  | float | The duration of this command in seconds | 

### moveByVelocity

Move the drone by controlling the velocity vector of the drone. A little bit of vx can make the drone move forwards, a little bit of vy can make it move sideways. A bit of vz can move the drone up or down vertically. The yaw_mode can set a specific yaw target, or tell the drone to move as a specified yaw rate. The yaw rate command is handy if you want to do a slow 360 and capture a nice smooth panorama. The duration says how long you want to apply these settings before reverting to a hover command. So you can say "fly forwards slowly for 1 second" using moveByVelocity(0.1, 0, 0, 1, ...). The cancelable_action can be used to canel all actions. In fact, every time you call another move* method you will automatically cancel any previous action that is happening.

| Parameter | Type    | Description                                                                                     | 
|-----------|---------|-------------------------------------------------------------------------------------------------| 
| vx        | float   | Velocity in x direction in meters per second                                                    | 
| vy        | float   | Velocity in y direction in meters per second                                                    | 
| vz        | float   | Velocity in z direction in meters per second                                                    | 
| yaw_mode  | YawMode | Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone | 

### moveByVelocityZ

Move the drone by controlling the velocity x,y of the drone but with a fixed altitude z. A little bit of vx can make the drone move forwards, a little bit of vy can make it move sideways. The yaw_mode can set a specific yaw target, or tell the drone to move as a specified yaw rate. The yaw rate command is handy if you want to do a slow 360 and capture a nice smooth panorama. The duration says how long you want to apply these settings before reverting to a hover command. So you can say "fly forwards slowly for 1 second" using moveByVelocityZ(0.1, 0, z, 1, ...). The cancelable_action can be used to canel all actions. In fact, every time you call another move* method you will automatically cancel any previous action that is happening

| Parameter | Type  | Description                            | 
|-----------|-------|----------------------------------------| 
| pitch     | float | pitch angle in degree                  | 
| roll      | float | roll angle in degree                   | 
| z         | float | z position in meters                   | 
| yaw       | float | the duration of this command in second | 
| duration  | float | target yaw angle in degrees            | 

### moveOnPath

Move the drone along the given path at the given speed and yaw. The lookahead argument will smooth this path by looking ahead from current location by a given number of meters, then it will try and move the drone to that lookahead position, thereby smoothing any corners in the path. The lookahead can also ensure the drone doesn't stop and start at each vertex along the path.

| Parameter          | Type             | Description                                                                                                                          | 
|--------------------|------------------|--------------------------------------------------------------------------------------------------------------------------------------| 
| path               | vector<Vector3r> | A series of x,y,z cordinates separated by commas, e.g. 0,0,-10,100,0,-10,100,100,-10,0,100,-10,0,0,-10 will fly a square box pattern | 
| velocity           | float            | The velocity in meters per second                                                                                                    | 
| drivetrain         | DrivetrainType   | Type of drive mode (1=forward only, 0= max degree of freedom)                                                                        | 
| yaw_mode           | YawMode          | Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone                                      | 
| lookahead          | float            | How far to look ahead on the path (default -1 means auto)                                                                            | 
| adaptive_lookahead | float            | Whether to apply adaptive lookahead (1=yes, 0=no)                                                                                    | 

### moveToPosition

Move the drone to the absolution x, y, z local positions at given speed and yaw. Remember z is negative. Positive z is under ground. Instead of moving to the yaw before starting, the drone will move to the yaw position smoothly as it goes, which means for short paths it may not reach that target heading until it has already reached the end point at which time the drone will continue rotating until it reaches the desired heading. The lookahead argument will smooth the path that moves the drone from it's current position to the target postion by looking ahead from current location by a given number of meters, it keeps doing this iteratively until it reaches the target position. This ensures a smoother flight.

| Parameter          | Type           | Description                                                                                     | 
|--------------------|----------------|-------------------------------------------------------------------------------------------------| 
| x                  | float          | x position in meters                                                                            | 
| y                  | float          | y position in meters                                                                            | 
| z                  | float          | z position in meters                                                                            | 
| velocity           | float          | the velocity to approach the position in meters per second                                      | 
| drivetrain         | DrivetrainType | Type of drive mode (1=forward only, 0= max degree of freedom)                                   | 
| yaw_mode           | YawMode        | Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone | 
| lookahead          | float          | How far to look ahead on the path (default -1 means auto)                                       | 
| adaptive_lookahead | float          | Whether to apply adaptive lookahead (1=yes, 0=no)                                               | 

### moveToZ

moveToZ is a shortcut for moveToPosition at the current x, y location.

| Parameter          | Type    | Description                                                                                     | 
|--------------------|---------|-------------------------------------------------------------------------------------------------| 
| z                  | float   | Move to specified z above launch position                                                       | 
| velocity           | float   | the velocity to approach the position in meters per second                                      | 
| yaw_mode           | YawMode | Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone | 
| lookahead          | float   | How far to look ahead on the path (default -1 means auto)                                       | 
| adaptive_lookahead | float   | Whether to apply adaptive lookahead (1=yes, 0=no)                                               | 

### moveByManual

Read current RC state and use it to control the vehicles.
Parameters sets up the constraints on velocity and minimum altitude while flying. If RC state is detected to violate these constraints then that RC state would be ignored.

| Parameter  | Type           | Description                                                                                                                                                             | 
|------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------| 
| vx_max     | float          | max velocity allowed in x direction                                                                                                                                     | 
| vy_max     | float          | max velocity allowed in y direction                                                                                                                                     | 
| vz_max     | float          | max velocity allowed in z direction                                                                                                                                     | 
| z_min      | float          | min z allowed for vehicle position                                                                                                                                      | 
| duration   | float          | after this duration vehicle would switch back to non-manual mode                                                                                                        | 
| drivetrain | DrivetrainType | when ForwardOnly, vehicle rotates itself so that its front is always facing the direction of travel. If MaxDegreeOfFreedom then it doesn't do that (crab-like movement) | 
| yaw_mode   | YawMode        | Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone                                                                         | 

### rotateToYaw

Rotate the drone to the specified fixed heading (yaw) while remaining stationery at the current x, y, and z.

| Parameter | Type  | Description        | 
|-----------|-------|--------------------| 
| yaw       | float | Degrees            | 
| margin    | float | How accurate to be | 

### rotateByYawRate

Rotate the drone to the specified yaw rate while remaining stationery at the current x, y, and z.

| Parameter | Type  | Description          | 
|-----------|-------|----------------------| 
| yaw_rate  | float | Degrees per second   | 
| duration  | float | Maximum time to wait | 

### hover

Hover at the current x, y, and z. If the drone is moving when this is called, it will try and move back to the location it was at when this command was received and hover there.

### setSafety

### setSafety

### getMultirotorState

### getPosition

get the current local position in NED coordinate (x=North/y=East,z=Down) so z is negative.

### getVelocity

Get the current velocity of the drone

### getOrientation

Get the current orientation (or attitude) of the drone as a Quaternion.

### getLandedState

Get debug pose, meaning of which is dependent on application usage. For example, this could be pose of real vehicle from log playback

### getRCData

Get the current RC inputs when RC transmitter is talking to to flight controller

### getGpsLocation

### isSimulationMode
