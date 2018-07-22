extern crate rtt;
extern crate rand;
extern crate kdvtree;
extern crate bit_vec;
extern crate pathfinding;
#[macro_use] extern crate itertools;

pub mod solver;
pub mod router;
pub mod coord;
pub mod state;
pub mod model;
pub mod cmd;
pub mod kd;

#[cfg(test)] mod junk;
