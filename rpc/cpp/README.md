# MsgPack-RPC in C++

This is a simple example program used to test the communication with the AirSim API.

## Prerequisite

- g++/gcc
- Have pthreads globally installed
- cmake
- make
- Unreal Engine + AirSim
- Have [rpclib](https://github.com/rpclib/rpclib) globally installed

## Compile

Run the following commands in the current directory (we're assuming you have rpclib installed globally).

```console
$ cmake .
$ make
```

## Run the program

Before running the following commands you must have AirSim running with the drone simulation.

```console
$ ./main
```
