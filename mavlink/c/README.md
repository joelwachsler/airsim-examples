# MAVLink in C

## Prerequisite

- gcc
- CMake
- Make
- [MAVLink v1 library](https://github.com/mavlink/c_library_v1)
- Linux operating system with POSIX sockets (sys/socket.h)
  - Works on windows with [Cygwin](https://www.cygwin.com/)

## Compile

Get MAVLink v1.0 by issuing the following commands:

```
$ git submodule update --init --recursive
```

To compile the program run the following commands from the current directory

```
$ cmake .
$ cmake . --build
```

## Run

To run he program execute the following commands in the current directory

```
$ ./mavlink # Linux
$ ./mavlink.exe # Windows
```

## Explanation

### MAVLink library

We used the first version of the C library for MAVLink which can be found [here](https://github.com/mavlink/c_library_v1). The MAVLink library has generated C code for various different message definitions but we only used those found under the XML file called [common.xml](https://github.com/mavlink/c_library_v1/blob/master/message_definitions/common.xml).

### Creating a connection to AirSim
AirSim listens and sends UDP messages on ports 14556 and 14560 by default. Thus the program starts by setting up a socket to listen to incoming UDP-messages from AirSim on port 14560.

```c
void socket_init() {
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    error("Opening socket");
  }

  memset(&server, 0, sizeof(struct sockaddr_in));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0) {
    error("Binding error");
  }
}
```

### Reading GPS messages
GPS Messages sent from MAVLink are retrieved in the following function:
```c
mavlink_hil_gps_t get_gps_msg() {
  while(1) {
    n = recvfrom(sockfd, buf, 1024, 0, (struct sockaddr *)&server, &server_len);
    if (n < 0) {
      error("Failed to recvfrom");
    }
    mavlink_message_t msg;
    mavlink_status_t status;
    unsigned int temp = 0;
    for (int i = 0; i < n; i++) {
      temp = buf[i];
      if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS) {
          mavlink_hil_gps_t gps;
          mavlink_msg_hil_gps_decode(&msg, &gps);
          return gps;
        }
      }
    }
  }
}
```
The function receives messages from the socket which was previously initialized and parses these messages. If a message is of the type [HIL_GPS](http://mavlink.org/messages/common#HIL_GPS) the function returns this message decoded otherwise it keeps repeating the same procedure.

### Update the PWM of the motors
The update of the PWM of each motor is done in the following function:
```c
void update_motor_pwm(float pwm[]) {
  mavlink_message_t msg;
  memset(buf, 0, MAVLINK_MAX_PACKET_LEN);

  const float controls[] = {
    pwm[0],
    pwm[1],
    pwm[2],
    pwm[3],
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f
  };

  size_t msglen = mavlink_msg_hil_actuator_controls_pack(0, 200, &msg, 0, controls, 0, 0);
  mavlink_msg_to_send_buffer(buf, &msg);
  n = sendto(sockfd, buf, msglen, 0, (struct sockaddr*)&server, &server_len);

  if (n < 0) {
    error("Failed to send message");
  }
}
```
This function takes an array of floats between -1 and 1 which represent the PWM of each individual motor. It will then pack these values together with other data necessary for the [HIL_ACTUATOR_CONTROLS](http://mavlink.org/messages/common#HIL_ACTUATOR_CONTROLS) and send it through the previously initialized socket.

### The hovering algorithm
This algorithm tries to keep the drone afloat at approximately 2.5 meters. It does so by constantly reading [HIL_GPS](http://mavlink.org/messages/common#HIL_GPS) messages for the current altitude, and depending on on the value of that sets the PWM of the motors on the drone appropriately by sending [HIL_ACTUATOR_CONTROLS](http://mavlink.org/messages/common#HIL_ACTUATOR_CONTROLS) messages.

It starts by getting the ground altitude and sets a target altitude to maintain.
```c
// Initialize our ground altitude and set a target altitude
int ground = get_gps_msg().alt;
int target_alt = 2500;
```
It then checks the currently altitude repeatedly in a loop and if the current altitude is below the target altitude it sets the PWM of the motors to 0.5 (which makes the drone lift with our settings), and if its above it sets the PWM to 0.48 (which makes the drone fall with our settings).

```c
while(1) {
  //Retrieve a gps message to get our current altitude
  mavlink_hil_gps_t gps = get_gps_msg();
  volatile int height = gps.alt;
  int diff = height - ground;
  printf("Current height: %fm\n", (float) diff / 1000.0);

  if (diff > target_alt) {
    pwm[0] = 0.48;
    pwm[1] = 0.48;
    pwm[2] = 0.48;
    pwm[3] = 0.48;
    printf("Updating the pwm to %f\n", 0.48);
  } else {
    pwm[0] = 0.50;
    pwm[1] = 0.50;
    pwm[2] = 0.50;
    pwm[3] = 0.50;
    printf("Updating the pwm to %f\n", 0.50);
  }
  update_motor_pwm(pwm);
}
```