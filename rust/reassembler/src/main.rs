extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

//mod optimizer;
//use optimizer::Optimizer;

use clap::Arg;
use std::fs::File;
use std::io::{Read,Write};
use std::collections::VecDeque;
use std::iter::DoubleEndedIterator;

use icfpc2018_lib as kernel;
use kernel::cmd::BotCommand;
use kernel::coord::{LinearCoordDiff,Axis,M,Coord,CoordDiff};

#[derive(Debug)]
enum Error {
    Args(clap::Error),
    Io(std::io::Error),
    Cmd(kernel::cmd::Error),
    ModelReadError(kernel::model::Error),
}




#[derive(Debug,Clone,Copy,Eq,PartialEq)]
enum Direction {
    Forward,
    Backward,
}
impl Direction {
    pub fn switch(&mut self) {
        if *self == Direction::Forward {
            *self = Direction::Backward;
        } else  {
            *self = Direction::Forward;
        }
    }
}

#[derive(Debug,Clone,Copy)]
struct BotTask {
    first: usize,
    count: usize,
}

#[derive(Debug,Clone,Copy)]
enum Delta {
    Full,
    Pair,
    One,
}
impl Delta {
    fn get_iter(&self) -> impl Iterator<Item = M> {
        match &self {
            Delta::Full => (-1 .. 2).into_iter(),
            Delta::Pair => (0 .. 2).into_iter(),
            Delta::One => (0 .. 1).into_iter(),
        }
    }
}

#[derive(Debug,Clone,Copy)]
enum Cmd { 
    XMove(M), // move to coord from current
    YMove(M),
    ZMove(M),
    Fill(Coord),
}


#[derive(Debug,Clone,Copy)]
struct Stripe {
    x: M,
    min_z: M,
    max_z: M,
    dx: Delta,
}
impl Stripe {
    pub fn get_z_iter(&self) -> impl DoubleEndedIterator<Item = M>{
        (self.min_z .. self.max_z + 1).into_iter()
    }
}

struct Translator<I> {
    bot_state: Coord,
    cmd_iter: I,
    buffer: VecDeque<BotCommand>,
}
impl<I> Translator<I> {
    pub fn new(bot_state: Coord, cmd_iter: I) -> Translator<I> {
        Translator {
            bot_state: bot_state,
            cmd_iter: cmd_iter,
            buffer: VecDeque::new(),
        }
    }
}
impl<I> Iterator for Translator<I>
    where I: Iterator<Item=Cmd>
{
    type Item = BotCommand;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.buffer.len()>0 {
                return self.buffer.pop_front();
            }
            match self.cmd_iter.next() {
                None => return None,
                Some(Cmd::XMove(x)) => {
                    let v = x - self.bot_state.x;
                    self.bot_state.x = x;
                    if v==0 { continue; }
                    let d = if v<0 { -1 } else { 1 };
                    let mut m = v.abs();
                    while m>0 {
                        self.buffer.push_back(BotCommand::smove(LinearCoordDiff::Long{ axis: Axis::X, value: if m > 15 { 15*d } else { m*d }}).unwrap());
                        m -= 15;
                    }
                },
                Some(Cmd::YMove(y)) => {
                    let v = y - self.bot_state.y;
                    self.bot_state.y = y;
                    if v==0 { continue; }
                    let d = if v<0 { -1 } else { 1 };
                    let mut m = v.abs();
                    while m>0 {
                        self.buffer.push_back(BotCommand::smove(LinearCoordDiff::Long{ axis: Axis::Y, value: if m > 15 { 15*d } else { m*d }}).unwrap());
                        m -= 15;
                    }
                },
                Some(Cmd::ZMove(z)) => {
                    let v = z - self.bot_state.z;
                    self.bot_state.z = z;
                    if v==0 { continue; }
                    let d = if v<0 { -1 } else { 1 };
                    let mut m = v.abs();
                    while m>0 {
                        self.buffer.push_back(BotCommand::smove(LinearCoordDiff::Long{ axis: Axis::Z, value: if m > 15 { 15*d } else { m*d }}).unwrap());
                        m -= 15;
                    }
                },
                Some(Cmd::Fill(c)) => return Some(BotCommand::Fill{ near: CoordDiff(c) }),
            }
        }
    }
}

struct Reverser {
    iter: std::vec::IntoIter<BotCommand>,
}
impl Reverser {
    pub fn new<I: Iterator<Item=BotCommand>>(iter: I) -> Reverser {
        Reverser {
            iter: iter.collect::<Vec<_>>().into_iter(),
        }
    }
}
impl Iterator for Reverser {
    type Item = BotCommand;
    
    fn next(&mut self) -> Option<Self::Item> {
        match self.iter.next_back() {
            None => None,
            Some(BotCommand::Wait) |
            Some(BotCommand::Halt) |
            Some(BotCommand::Flip) |
            Some(BotCommand::GFill{ .. }) |
            Some(BotCommand::GVoid{ .. }) |
            Some(BotCommand::FusionP{ .. }) |
            Some(BotCommand::FusionS{ .. }) |
            Some(BotCommand::Fission{ .. }) => unimplemented!(),
            Some(BotCommand::Fill{ near }) => Some(BotCommand::Void{ near }),
            Some(BotCommand::Void{ near }) => Some(BotCommand::Fill{ near }),
            Some(BotCommand::SMove{ long: LinearCoordDiff::Long{ axis, value} }) => Some(BotCommand::SMove{ long: LinearCoordDiff::Long{ axis: axis, value: -value } }),
            Some(BotCommand::LMove{ short1: LinearCoordDiff::Short { axis: a1, value: v1}, short2: LinearCoordDiff::Short { axis: a2, value: v2} }) => {
                Some(BotCommand::LMove{ short1: LinearCoordDiff::Short { axis: a2, value: -v2}, short2: LinearCoordDiff::Short { axis: a1, value: -v1} })
            },
            _ => unreachable!(),
        }
    }
}

fn main() -> Result<(),Error> {
    let app = app_from_crate!()
        .arg(Arg::with_name("source")
             .display_order(1)
             .short("s")
             .long("source")
             .help("Source trace (In)",)
             .takes_value(true))
        .arg(Arg::with_name("target")
             .display_order(2)
             .short("t")
             .long("target")
             .help("Target trace (In)")
             .takes_value(true))
        .arg(Arg::with_name("trace")
             .display_order(3)
             .short("o")
             .long("out")
             .help("Optimized trace (Out)")
             .takes_value(true));

    let matches = app.get_matches();
    let source = value_t!(matches, "source", String).map_err(Error::Args)?;
    let target = value_t!(matches, "target", String).map_err(Error::Args)?;
    let trace = value_t!(matches, "trace", String).map_err(Error::Args)?;



    let mut f = File::open(&source).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;
    let mut destroy_cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;

    let mut f = File::open(&target).map_err(Error::Io)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).map_err(Error::Io)?;
    let create_cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;

    destroy_cmds.pop();

    let asc = destroy_cmds.into_iter().chain(create_cmds.into_iter()).collect();
  
    {
        let buffer = kernel::cmd::into_bytes(&asc).unwrap();
        let mut f = File::create(&trace).map_err(Error::Io)?;
        f.write_all(&buffer).map_err(Error::Io)?;
    }

    Ok(())
}

