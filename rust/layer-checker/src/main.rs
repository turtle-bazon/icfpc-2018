extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

mod optimizer;
use optimizer::Optimizer;

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
    ModelReadError(kernel::model::Error),
    Cmd(kernel::cmd::Error),
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

fn main() -> Result<(),Error> {
    let mut app = app_from_crate!()
        .arg(Arg::with_name("original")
             .display_order(1)
             .short("i")
             .long("in")
             .help("Original trace (In)",)
             .takes_value(true))
        .arg(Arg::with_name("optimized")
             .display_order(2)
             .short("o")
             .long("out")
             .help("Optimized trace (Out)")
             .takes_value(true));

    let matches = app.get_matches();
    let original = value_t!(matches, "original", String).map_err(Error::Args)?;
    let optimized = value_t!(matches, "optimized", String).map_err(Error::Args)?;

    
    let matrix = kernel::model::read_model_file(&original).map_err(Error::ModelReadError)?;
    let mut iter = matrix.filled_voxels();
    let (mut min, mut max) = match iter.next() {
        Some(c) => (c.clone(),c.clone()),
        None => panic!("No matter!"),
    };
    for c in iter {
        if c.x<min.x { min.x = c.x }
        if c.y<min.y { min.y = c.y }
        if c.z<min.z { min.z = c.z }
        if c.x>max.x { max.x = c.x }
        if c.y>max.y { max.y = c.y }
        if c.z>max.z { max.z = c.z }
    }
    println!("Dim: {:?}",matrix.dim());
    println!("Min: {:?}",min);
    println!("Max: {:?}",max);

    let mut width = max.x - min.x + 1;
    let mut stripes = Vec::new();
    let mut x = min.x;
    while width > 0 {
        if width >= 3 {
            stripes.push(Stripe {
                x: x + 1,
                min_z: min.z,
                max_z: max.z,
                dx: Delta::Full,
            });
            x += 3;
        } else {
            stripes.push(Stripe {
                x: x,
                min_z: min.z,
                max_z: max.z,
                dx: if width==2 { Delta::Pair } else { Delta::One },
            });
        }
        width -= 3;
    }

    let mut ocount = Vec::new();
    for _ in 0 .. stripes.len() {
        ocount.push(0);
    }
    for y in min.y .. max.y + 1 {
        for (si,s) in stripes.iter().enumerate() {
            let mut oc = 0;
            for z in s.min_z .. s.max_z + 1 {
                let mut k = 0;
                for dx in s.dx.get_iter() {
                    if matrix.is_filled(&Coord{ x: s.x + dx, y: y, z: z }) { k += 1; }
                }
                if k>0 { k += 1; }
                oc += k;
            }
            ocount[si] += oc;
        }
    }
    for (si,s) in stripes.iter().enumerate() {
        println!("{:?} {}",s,ocount[si]);
    }

    let bot_config = {
        let mut cnts = vec![0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,];
        let m = cnts.len();
        for (i,_) in stripes.iter().enumerate() {
            cnts[i%m] += 1;
        }
        let mut p = 0;
        let mut res = Vec::new();
        for c in cnts {
            res.push(BotTask{ first: p, count: c });
            p += c;
        }
        res
    };
    
    //let bot_config = vec![BotTask{ first: 0, count: 6 }];
    /*let bot_config = vec![BotTask{ first: 0, count: 2 },
                          BotTask{ first: 2, count: 1 },
                          BotTask{ first: 3, count: 1 },
                          BotTask{ first: 4, count: 2 }];*/

    /* create */
    let mut states = Vec::new();
    let mut commands = Vec::new();
    for _ in &bot_config {
        commands.push(Vec::new());
    }
    for (ibot,bot) in bot_config.iter().enumerate() {
        let cmds: &mut Vec<Cmd> = &mut commands[ibot];
        states.push(Coord{ x: stripes[bot.first].x, y: 0, z: 0});
        let mut layer_direction = Direction::Forward;
        let mut stripe_direction = Direction::Forward;
        for y in min.y .. max.y + 1 {
            cmds.push(Cmd::YMove(y+1));
            let mut iter = (bot.first .. bot.first + bot.count).into_iter();
            loop {
                let si = match match layer_direction {
                    Direction::Forward => iter.next(),
                    Direction::Backward => iter.next_back(),
                } {
                    None => break,
                    Some(si) => si,
                };
                let s = stripes[si];
                cmds.push(Cmd::XMove(s.x));
                let mut z_iter = s.get_z_iter();
                loop {
                    let z = match match stripe_direction {
                        Direction::Forward => z_iter.next(),
                        Direction::Backward => z_iter.next_back(),
                    } {
                        None => break,
                        Some(z) => z,
                    };
                    cmds.push(Cmd::ZMove(z));

                    for dx in s.dx.get_iter() {
                        if matrix.is_filled(&Coord{ x: s.x + dx, y: y, z: z }) {
                            cmds.push(Cmd::Fill(Coord{ x: dx, y: -1, z: 0}));
                        }
                    }
                    
                }
                stripe_direction.switch();
            }

            layer_direction.switch();
        }
        cmds.push(Cmd::ZMove(0));
        cmds.push(Cmd::XMove(stripes[bot.first].x));
        cmds.push(Cmd::YMove(0));
    }
    /*for (i,vcmd) in commands.iter().enumerate() {
        println!("Bot: {}",i);
        for c in vcmd {
            println!("{:?}",c);
        }
    }*/

    /* spawn creators*/
    //let max_bots = 39;
    //if bot_config.len()==1 {
    //    let mut asc = Translator::new(Coord{x:0,y:0,z:0},vec![Cmd::XMove(stripes[0].x)].into_iter()).collect::<Vec<_>>();
    //}
    let mut max_bots = 39;
    let mut n_bots = 1;
    let mut asc = Vec::new();
    for (ibot,bot) in bot_config.iter().enumerate() {
        let mut pos = Coord{x:0,y:0,z:0};
        if ibot>0 {
            for _ in 1 .. n_bots { asc.push(BotCommand::wait().unwrap()); }
            asc.push(BotCommand::fission(CoordDiff(Coord{x:1,y:0,z:0}),max_bots-1).unwrap());
            max_bots -= 1;
            n_bots += 1;
            pos = Coord{x:stripes[bot_config[ibot-1].first].x+1,y:0,z:0};
        }
        for c in Translator::new(pos,vec![Cmd::XMove(stripes[bot.first].x)].into_iter()) {
            for _ in 1 .. n_bots { asc.push(BotCommand::wait().unwrap()); }
            asc.push(c);
        }
    }
    
    for _ in 1 .. n_bots { asc.push(BotCommand::wait().unwrap()); }
    asc.push(BotCommand::flip().unwrap());
    
    /* proc */
    let mut v = VecDeque::new();
    for _ in bot_config.iter().enumerate() {
        v.push_front(Optimizer::new(Translator::new(states.pop().unwrap(),commands.pop().unwrap().into_iter())));
    }
    loop {
        let mut cnt = 0;
        let mut step = Vec::new();
        for v in &mut v {
            step.push(match v.next() {
                None => BotCommand::wait().unwrap(),
                Some(c) => {
                    cnt += 1;
                    c
                },
            })
        }
        if cnt==0 { break; }
        asc.extend(step.into_iter());
    }
    //asc.extend(Translator::new(states.pop().unwrap(),commands.pop().unwrap().into_iter()));  

    /* join */
    asc.push(BotCommand::flip().unwrap());
    for _ in 1 .. n_bots { asc.push(BotCommand::wait().unwrap()); }
    //asc.extend(Translator::new(Coord{x:stripes[0].x,y:0,z:0},vec![Cmd::XMove(0)].into_iter()));
    let mut n = bot_config.len()-1;
    while n>0 { 
        for c in Translator::new(Coord{x:stripes[bot_config[n].first].x,y:0,z:0},vec![Cmd::XMove(stripes[bot_config[n-1].first].x+1)].into_iter()) {
            for _ in 1 .. n_bots { asc.push(BotCommand::wait().unwrap()); }
            asc.push(c);
        }
        for _ in 1 .. n_bots-1 { asc.push(BotCommand::wait().unwrap()); }
        asc.push(BotCommand::pfusion(CoordDiff(Coord{x:1,y:0,z:0})).unwrap());
        asc.push(BotCommand::sfusion(CoordDiff(Coord{x:-1,y:0,z:0})).unwrap());
        max_bots += 1;
        n_bots -= 1;
        n -= 1;
    }
    for c in Translator::new(Coord{x:stripes[bot_config[0].first].x,y:0,z:0},vec![Cmd::XMove(0)].into_iter()) {
        asc.push(c);
    }
    asc.push(BotCommand::halt().unwrap()); 

    let mut cnt = 0;
    for c in &asc {
        println!("{:?}",c);
        cnt += 1;
    }
    println!("Count: {}",cnt);
  
    {
        let buffer = kernel::cmd::into_bytes(&asc).unwrap();
        let mut f = File::create(&optimized).map_err(Error::Io)?;
        f.write_all(&buffer).map_err(Error::Io)?;
    }

    Ok(())
}


#[cfg(test)]
mod test {
    use super::*;
    
    #[test]
    fn test_opt_moves1() {
        let mut v = vec![
            Move{ axis: Axis::X, value: 5 },
            Move{ axis: Axis::Y, value: 1 },
            Move{ axis: Axis::X, value: -5 },
            ];
        println!("");
        println!("{:?}",v);
        optimize_moves(&mut v);
        println!("{:?}",v);
        assert_eq!(1,0);
    }

    #[test]
    fn test_opt_moves2() {
        let mut v = vec![
            Move{ axis: Axis::X, value: -3 },
            Move{ axis: Axis::Y, value: 1 },
            Move{ axis: Axis::X, value: 5 },
            ];
        println!("");
        println!("{:?}",v);
        optimize_moves(&mut v);
        println!("{:?}",v);
        assert_eq!(1,0);
    }

    #[test]
    fn test_opt_moves3() {
        let mut v = vec![
            Move{ axis: Axis::X, value: -6 },
            Move{ axis: Axis::Y, value: 1 },
            Move{ axis: Axis::X, value: 5 },
            ];
        println!("");
        println!("{:?}",v);
        optimize_moves(&mut v);
        println!("{:?}",v);
        assert_eq!(1,0);
    }

    #[test]
    fn test_opt_moves4() {
        let mut v = vec![
            Move { axis: Axis::Z, value: 2 },
            Move { axis: Axis::X, value: 1 },
            Move { axis: Axis::Z, value: -7 },
            Move { axis: Axis::Y, value: 1 },
            Move { axis: Axis::Z, value: 7 },
            Move { axis: Axis::X, value: -1 },
            Move { axis: Axis::Z, value: -2 }
            ];
        println!("");
        println!("{:?}",v);
        optimize_moves(&mut v);
        println!("{:?}",v);
        assert_eq!(1,0);
    }


}
