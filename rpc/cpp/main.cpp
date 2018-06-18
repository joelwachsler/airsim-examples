#include <iostream>
#include "rpc/client.h"

int main()
{
  // Creating a client that connects to the localhost on port 41451
  rpc::client client("127.0.0.1", 41451);

  // Tell the controller to enable api contrls
  client.call("enableApiControl", true);
  // Arm the AirSim drone -> turn on motors
  client.call("armDisarm", true);
  // Go 5m up in the air
  client.call("takeoff", 5.0);
  std::this_thread::sleep_for(std::chrono::duration<double>(5));
  // Hover at the current altitude
  client.call("hover");
  return 0;
}