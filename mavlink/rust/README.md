# MAVLink example using Rust

## Prerequisite

- Rust
- Cargo
- AirSim

## Compile and run

In the current directory run the following

```console
$ cargo run
```

## Explanation

### MAVLink library

We forked a third party Rust library called [mavlink](https://crates.io/crates/mavlink). This library generates Rust code from MAVLink messages defined in XML format from a file called [common.xml](https://github.com/3drobotics/rust-mavlink/blob/master/common.xml) and also handles the setup of the UDP connection to AirSim or any other UDP + MAVLink compatible device. The reason for the fork is because this library is missing a vital message called [HIL_ACTUATOR_CONTROLS](http://mavlink.org/messages/common#HIL_ACTUATOR_CONTROLS) which is necessary for us to send PWM messages to the motors of the drone. The fork can be found [here](https://github.com/JoelWachsler/rust-mavlink) and the only thing that's changed is the addition of HIL_ACTUATOR_CONTROLS message definition which can be seen here:

```xml
<message id="93" name="HIL_ACTUATOR_CONTROLS">
    <description>Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)</description>
    <field type="uint64_t" name="time_usec" units="us">Timestamp (microseconds since UNIX epoch or microseconds since system boot)</field>
    <field type="float[16]" name="controls">Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.</field>
    <field type="uint8_t" name="mode" enum="MAV_MODE">System mode (MAV_MODE), includes arming state.</field>
    <field type="uint64_t" name="flags" display="bitmask">Flags as bitfield, reserved for future use.</field>
</message>
```

and a simple update of [mavlink-dump.rs](https://github.com/JoelWachsler/rust-mavlink/blob/master/src/bin/mavlink-dump.rs) showing that the motors can be controlled using the HIL_ACTUATOR_CONTROLS message.

### Create a connection to AirSim

The program starts by creating a new UDP connection to localhost on port 14560 (which is one of the ports AirSim is listening for MAVLink messages on). This connection is then saved as a thread-safe reference in the `vehicle` variable.

```rust
let vehicle = Arc::new(mavlink::connect("udpin:127.0.0.1:14560").unwrap());
```
### Heartbeats

[Heartbeats](http://mavlink.org/messages/common#HEARTBEAT) aren't necessary in our case because the settings PX4 settings which we're using are already defined from our settings.json. The different settings which can be set from heartbeat messages can be seen in the [Airsim source code](https://github.com/Microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/controllers/MavLinkDroneController.hpp#L567).

### Reading GPS messages

Raw GPS data is read by using the following function:

```rust
/**
 * Blocks until a HIL_GPS message is received.
 */
fn get_gps_msg(
    vehicle: &Arc<Box<mavlink::MavConnection + std::marker::Send + std::marker::Sync>>,
) -> common::HIL_GPS_DATA {
    loop {
        match vehicle.recv().unwrap() {
            MavMessage::HIL_GPS(gps_data) => return gps_data,
            _ => continue,
        }
    }
}
```

This function uses the provided `vehicle` variable which contains a reference to the MAVLink UDP connection to listen for incoming MAVLink messages. If the incoming messages is a [HIL_GPS](http://mavlink.org/messages/common#HIL_GPS) the function returns otherwise it will block until a HIL_GPS is received.

### Update the PWM of the motors

Updating the PWM of the motors is done by sending [HIL_ACTUATOR_CONTROLS](http://mavlink.org/messages/common#HIL_ACTUATOR_CONTROLS) messages. An abstraction of this has been made by defining the `update_motor_pwm` function which is defined as follows:

```rust
/**
 * Updates the pvm of the four motor controls.
 */
fn update_motor_pwm(
    motor_pwm: [f32; 4],
    vehicle: &Arc<Box<mavlink::MavConnection + std::marker::Send + std::marker::Sync>>,
) {
    let controls: Vec<f32> = vec![
        motor_pwm[0],
        motor_pwm[1],
        motor_pwm[2],
        motor_pwm[3],
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
    ];

    let control_msg = MavMessage::HIL_ACTUATOR_CONTROLS(common::HIL_ACTUATOR_CONTROLS_DATA {
        time_usec: 0,
        flags: 0,
        controls,
        mode: 0,
    });

    vehicle.send(&control_msg).ok();
}
```

This function will take the provided `motor_pwm` array which defines the PWM of each motor, pack it in a HIL_ACTUATOR_CONTROL message and send it off to AirSim using the `send` method provided by the UDP connection provided by the `vehicle` variable.

### The hovering algorithm

The core of this algorithm works by getting [HIL_GPS](http://mavlink.org/messages/common#HIL_GPS) messages which tells us various GPS information about our drone, like for example the altitude. The algorithm then reacts to these messages by issuing [HIL_ACTUATOR_CONTROLS](http://mavlink.org/messages/common#HIL_ACTUATOR_CONTROLS) messages which is used to update the PWM of each motor on the drone.

The algorithm starts by getting the ground GPS altitude by waiting for which is used to determine the relative height needed to reach the target altitude.

```rust
// Let's initialize the ground variable
let ground: i32 = get_gps_msg(&vehicle).alt;
// Altitude, in meters * 1000 (positive for up)
let target_alt = 2500;
```

Now for the actual algorithm which will just wait until a HIL_GPS message is received. If the current relative altitude of that message is below the target altitude the PWM of the motors is set to 0.50 (which will make the drone go up) and if it's above the PWM will be set to 0.48 (which will make the drone go down).

```rust
// Takes off and tries to get an altitude of 2.5m
loop {
    // Blocking until a MAVLink message is received
    let height = get_gps_msg(&vehicle).alt;
    let diff = height - ground;

    // Variable for holding the current motor pwm.
    let pwm: f32;

    // Print formatting: 2500 -> 2.50
    println!("Current height: {:.2}m", diff as f32 / 1000.0);

    if diff > target_alt {
        pwm = 0.48;
    } else {
        pwm = 0.50;
    }

    println!("Updating the pwm to {:.2}", pwm);
    update_motor_pwm([pwm; 4], &vehicle);
}
```
