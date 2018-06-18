# MAVLink in C

## Prerequisite

- gcc
- CMake
- Make
- Linux operating system with POSIX sockets (sys/socket.h)
  - Works on windows with [Cygwin](https://www.cygwin.com/)

## Initialize submodules & Compile

In order to compile the project you need to initialize the submodules (fetches MAVLink)

```console
$ git submodule init
$ git submodule update
```

To build the project run the following command in the current directory.

```console
$ cmake . --build
```

## Run

To run he program execute the following commands in the current directory

```console
$ ./demo # Linux
$ ./demo.exe # Windows
```
