//! Linux i2c demo

extern crate linux_embedded_hal;
extern crate qmc5883l;

use qmc5883l::*;

use std::env;
use std::f32::consts::PI;
use std::process;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        println!("usage: {} /dev/i2c-N declination_radians", args[0]);
        process::exit(1);
    }
    let mut i2c_dev = linux_embedded_hal::I2cdev::new(&args[1]).unwrap();
    // Need correct magnetic declination for your location for accurate
    // readings. See http://www.magnetic-declination.com/
    let declination_rads = args[2].parse::<f32>().unwrap();
    QMC5883L::probe(&mut i2c_dev).expect("Expected qmc5883l");
    let mut dev = QMC5883L::new(i2c_dev);
    dev.continuous().unwrap();
    loop {
        let (x, y, z) = dev.read_magnetism().unwrap();
        let mut heading = (y as f32).atan2(x as f32) + declination_rads;
        if heading < 0.0 {
            heading += 2.0 * PI;
        } else if heading > 2.0 * PI {
            heading -= 2.0 * PI;
        }
        let heading_degrees = heading * 180.0 / PI;
        println!(
            "x={:.*}, y={:.*}, z={:.*}: heading={:.*} degrees",
            1, x, 1, y, 1, z, 1, heading_degrees
        );
    }
}
