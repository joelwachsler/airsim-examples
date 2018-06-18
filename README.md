# AirSim examples
This repository is the result of the bachelor thesis by Joel Wachsler and Daniel Aros Banda which ended in the summer of 2018. This repository contains code examples of how to communicate with AirSim by using the default API accessible through RPC and another message oriented API called MAVLink. It also contains a simple implementation of a flight controller using PID controllers in Rust and C.

## Setting up AirSim

AirSim can be run on Windows, Linux or Mac but Unreal Engine must be built from source on Linux which may take a lot of time.

### Recommended hardware

According to the Unreal [wiki](https://wiki.unrealengine.com/Recommended_Hardware) a powerful setup is needed to run Unreal Engine in development mode and as of current the following hardware is recommended:

- Desktop PC or Mac
- Windows 7 64-bit or Mac OS X 10.9.2 or later
- Quad-core Intel or AMD processor, 2.5 GHz or faster
- NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series card or higher
- 8 GB RAM

### Windows 10 setup (Recommended)

Using Windows is by far the easiest way to get up and running with Unreal Engine because there are pre-compiled binaries available.

Follow the instructions of how to build on Windows posted on AirSim's [github](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) page and you should be set.

A helpful video how to get started made by Chris Lovett can be seen in the video below:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=1oY8Qu5maQQ" target="_blank"><img src="http://img.youtube.com/vi/1oY8Qu5maQQ/0.jpg" alt="Unreal AirSim Setup from scratch" /></a>

### Linux setup (Ubuntu LTS 16.04)

We've noticed that the [Nouveau](https://nouveau.freedesktop.org/wiki/) open source drivers for NVidia does not work with Unreal Engine in Ubuntu and therefore the proprietary drivers from NVidia must be used. Instructions how to install them can be found [here](https://gist.github.com/wangruohui/df039f0dc434d6486f5d4d098aa52d07).

Follow the instructions of how to build on Linux posted on AirSim's [github](https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md) page and you should be set.

### Settings and APIs

Settings for AirSim are stored in `~/Document/AirSim/settings.json` after its first run and will contain the following:

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.0
}
```

This setup is fine if you're going to use the official API which can be found [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md). As of writing this the API documentation is quite lacking and therefore we've done you the favour of summarizing them [here](code/rpc/README.md). Example programs in C++ and Rust can also be found [here](code/rpc/).

The API mentioned above is fine if absolute control of the motors is not required but if that is not the case the lower level API accessible through [MAVLink](http://qgroundcontrol.org/mavlink/start) is required. To be able to access this API the following settings has to be used:

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.0,
  "DefaultVehicleConfig": "PX4",
  "PX4": {
      "UseSerial": false
  }
}
```

These settings allows us to contact AirSim with MAVLink over UDP on its default ports defined as 14560 and 14556. Because AirSim doesn't provide any documentation on how to use this API we've created some sample programs and API documentation which can be found [here](code/mavlink).
