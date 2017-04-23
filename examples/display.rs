extern crate dht22_pi;

use dht22_pi::read;

pub fn main() {
    let result = read(14);

    println!("{:?}", result);
}
