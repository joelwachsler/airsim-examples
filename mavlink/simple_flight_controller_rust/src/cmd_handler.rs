use std::io::{self, stdout, Write};
use std::sync::Arc;

use Drone;

pub fn start_cmd_handler(drone: Arc<Drone>) {
  println!("--- COMMAND_HANDLER STARTED ---");
  let mut running = true;

  while running {
    print!("> ");
    stdout().flush().unwrap();

    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    // Remove newline
    input.pop();
    // Remove return
    input.pop();

    let cmd: Vec<&str> = input.split(" ").collect();

    match cmd[0] {
      "arm" => {
        println!("Arming the drone!");
        drone.arm();
      },
      "disarm" => {
        println!("Disarming the drone!");
        drone.disarm();
      },
      "gohome" => {
        println!("Going to the original position!");
        let (lat, lon) = drone.get_home_coordinates();
        drone.set_target_coordinates(lat, lon);
      },
      "sethome" => {
        let (lat, lon) = drone.get_current_coordinates();
        println!("Setting new home coordinates (lat: {}, lon: {})", lat, lon);
        drone.set_home_coordinates(lat, lon);
      },
      "land" => {
        println!("Landing the drone! (going to the starting altitude)");
        drone.set_target_alt(0);
      },
      "gps" => {
        let (lat, lon) = drone.get_current_coordinates();
        println!("lat: {}, lon: {}", lat, lon);
      },
      "alt" => println!("Current altitude: {:.2}m", drone.get_current_alt() as f32 / 1000.0),
      "flyto" | "goto" => {
        let (lat, lon) = (cmd[1].parse::<f64>().unwrap(), cmd[2].parse::<f64>().unwrap());
        drone.set_target_coordinates(lat, lon);
      },
      "takeoff" => {
        let alt = cmd[1].parse::<f32>().unwrap();
        println!("Taking off to: {:.2}m", alt);
        drone.set_target_alt((alt * 1000.0) as i32);
      },
      "exit" | "q" => {
        running = false;
        println!("Thank you for flying!");
      },
      _ => println!("Invalid command!"),
    }
  }
}
