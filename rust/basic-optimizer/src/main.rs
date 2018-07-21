extern crate icfpc2018_lib;
#[macro_use] extern crate clap;

use clap::Arg;
use std::fs::File;
use std::io::{Read,Write};
use std::collections::VecDeque;

use icfpc2018_lib as kernel;
use kernel::cmd::BotCommand;
use kernel::coord::{LinearCoordDiff,Axis,M,Coord};

#[derive(Debug)]
enum Error {
    Args(clap::Error),
    Io(std::io::Error),
    ModelReadError(kernel::model::Error),
    Cmd(kernel::cmd::Error),
}

#[derive(Debug,Clone,Copy)]
struct Move {
    axis: Axis,
    value: M,
}
impl Move {
    fn from_linear(l: &LinearCoordDiff) -> Move {
        Move {
            axis: l.get_axis(),
            value: l.get_value(),
        }
    }
    fn is_lld(&self) -> bool {
        return (self.value>=-15)&&(self.value<=15);
    }
    fn to_llds(&self) -> Vec<LinearCoordDiff> {
        let dest = if self.value>0 {1} else {-1};
        let mut aval = self.value.abs();
        let mut res = Vec::new();
        while aval>0 {
            if aval>15 {
                res.push(LinearCoordDiff::Long{ axis: self.axis, value: dest*15 });
            } else {
                res.push(LinearCoordDiff::Long{ axis: self.axis, value: dest*aval });
            }
            aval -= 15;
        }
        res
    }
    fn to_lld(&self) -> LinearCoordDiff {
        LinearCoordDiff::Long{ axis: self.axis, value: self.value }
    }
    fn is_sld(&self) -> bool {
        return (self.value>=-5)&&(self.value<=5);
    }
    fn to_sld(&self) -> LinearCoordDiff {
        LinearCoordDiff::Short{ axis: self.axis, value: self.value }
    }
}

fn optimize_moves(movings: &mut Vec<Move>) {
    fn get_diff(v1: M, v2: M) -> M {
        if v1.abs()<=v2.abs() { v1 } else { -v2 }
    }
    
    let mut opt = true;
    while opt {
        opt = false;
        for i in (1 .. movings.len()-1) {
            if ((movings[i].value == 1)||(movings[i].value == -1))&&
                (movings[i-1].axis == movings[i+1].axis)&&((movings[i-1].value * movings[i+1].value)<0)
            {
                let d =  get_diff(movings[i-1].value,movings[i+1].value);
                movings[i-1].value -= d;
                movings[i+1].value += d;
                opt = true;
            }
        }
        if opt {
            let mut tmp = movings.iter().cloned().filter(|mv|mv.value!=0).collect::<Vec<_>>();
            std::mem::swap(&mut tmp, movings);
        }
        let mut sopt = false;
        for i in (0 .. movings.len()-1) {
            if movings[i].axis == movings[i+1].axis {
                movings[i+1].value += movings[i].value;
                movings[i].value = 0;
                sopt = true;
            }
        }
        if sopt {
            let mut tmp = movings.iter().cloned().filter(|mv|mv.value!=0).collect::<Vec<_>>();
            std::mem::swap(&mut tmp, movings);
        }
    }
}

fn optimize_lld_pairs(cmds: &mut Vec<BotCommand>) {
    loop {
        let mut tmp = None;
        for i in (0 .. cmds.len()-1) {
            match (cmds[i],cmds[i+1]) {
                (BotCommand::SMove{ long: long1 },BotCommand::SMove{ long: long2 }) if (long1.get_value().abs()<=5)&&(long2.get_value().abs()<=5) => {
                    let short1 = LinearCoordDiff::Short{
                        axis: long1.get_axis(),
                        value: long1.get_value(),
                    };
                    let short2 = LinearCoordDiff::Short{
                        axis: long2.get_axis(),
                        value: long2.get_value(),
                    };
                    tmp = Some((i,BotCommand::lmove(short1,short2).unwrap()));
                    
                    break;    
                },
                (_,_) => continue,
            }
        }
        match tmp {
            None => break,
            Some((idx,cmd)) => {
                cmds.remove(idx);
                cmds[idx] = cmd;
            }
        }
    }
}

struct Optimizer<I> {
    cmds: I,
    buffer: VecDeque<BotCommand>,
    movings: Vec<Move>,
}
impl<I> Optimizer<I> {
    pub fn new(iter: I) -> Optimizer<I> {
        Optimizer {
            cmds: iter,
            buffer: VecDeque::new(),
            movings: Vec::new(),
        }
    }
    pub fn add_move(&mut self, d: &LinearCoordDiff) {
        let mov = Move::from_linear(d);
        if self.movings.len()>0 {
            let idx = self.movings.len()-1;        
            if self.movings[idx].axis == mov.axis {
                self.movings[idx].value += mov.value
            } else {
                self.movings.push(mov)
            }
        } else {
            self.movings.push(mov)
        }
    }
    
    pub fn flush_moves(&mut self) -> impl Iterator<Item = BotCommand> {
        optimize_moves(&mut self.movings);
        let mut res = Vec::new();
        for m in self.movings.drain(0..) {
            for lld in m.to_llds() {
                res.push(BotCommand::smove(lld).unwrap());
            }
        }
        optimize_lld_pairs(&mut res);
        res.into_iter()
    }
}
impl<I> Iterator for Optimizer<I>
    where I: Iterator<Item = BotCommand>
{
    type Item = BotCommand;
    fn next(&mut self) -> Option<Self::Item> {        
        loop {
            if self.buffer.len()>0 {
                return self.buffer.pop_front();
            }
            match self.cmds.next() {
                None => return None,
                c @ Some(BotCommand::Wait) => continue,
                c @ Some(BotCommand::Halt) |
                c @ Some(BotCommand::Flip) |
                c @ Some(BotCommand::Fill{ .. }) |
                c @ Some(BotCommand::FusionP{ .. }) |
                c @ Some(BotCommand::FusionS{ .. }) |
                c @ Some(BotCommand::Fission{ .. }) => {
                    if self.movings.len()>0 {
                        let moves = self.flush_moves();
                        self.buffer.extend(moves);
                        self.buffer.push_back(c.unwrap());
                    } else {
                        return c;
                    }
                },
                Some(BotCommand::SMove{ long }) => {
                    self.add_move(&long);
                },
                Some(BotCommand::LMove{ short1, short2 }) => {
                     self.add_move(&short1);
                     self.add_move(&short2);  
                },
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

    
    /*let matrix = kernel::model::read_model_file("../../problems/LA001_tgt.mdl").map_err(Error::ModelReadError)?;

    let c = kernel::coord::Coord{ x:1, y:1, z:1 };
    for n in c.get_neighbours() {
        println!("{:?}",n);
}*/

    let mut f = File::open(&original).map_err(Error::Io)?;
    let mut buffer = Vec::new();

    f.read_to_end(&mut buffer).map_err(Error::Io)?;
    let cmds = kernel::cmd::from_bytes(&buffer).map_err(Error::Cmd)?;
    let old_len = cmds.len();
    
    let opt_cmds = Optimizer::new(cmds.into_iter()).collect::<Vec<_>>();
    let mut new_len = 0;
    buffer = kernel::cmd::into_bytes(&opt_cmds).unwrap();
    for c in  opt_cmds {
        //print!("{:?}\n",c);
        new_len += 1;
    }
    println!("Old: {}, new: {}",old_len,new_len);

    {
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
