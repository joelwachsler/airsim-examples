extern crate futures;
extern crate rmp_rpc;
extern crate tokio_core;

use std::net::SocketAddr;
use std::{thread, time};

use futures::Future;
use rmp_rpc::Client;
use tokio_core::net::TcpStream;
use tokio_core::reactor::Core;

fn main() {
    // Create a new tokio event loop to run the client
    let mut core = Core::new().unwrap();

    let addr: SocketAddr = "127.0.0.1:41451".parse().unwrap();
    let handle = core.handle();

    // Connect to the AirSim simple flight controller via Messagepack-RPC over TCP
    let client = TcpStream::connect(&addr, &handle)
        .or_else(|e| {
            println!("I/O error in the client: {}", e);
            Err(())
        })
        .and_then(move |stream| {
            let client = Client::new(stream, &handle);

            client.request("enableApiControl", &[true.into()]);
            client.request("armDisarm", &[true.into()]);
            client.request("takeoff", &[(5.0).into()]);
            thread::sleep(time::Duration::from_secs(5));
            client.request("hover", &[]).and_then(|_res| Ok(()))
        });

    // Run the client and print the result
    match core.run(client) {
        Ok(_) => println!("Client finished successfully"),
        Err(_) => println!("Client failed"),
    }
}
