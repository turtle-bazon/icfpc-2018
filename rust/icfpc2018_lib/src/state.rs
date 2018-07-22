use std::collections::{HashSet, BTreeMap};
use std::mem;

use super::{
    coord::{
        Coord,
        Matrix,
        Region,
    },
    cmd::{
        BotCommand,
    },
};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Harmonics {
    Low,
    High,
}

pub type Bid = usize;

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Bot {
    pub pos: Coord,
    pub seeds: Vec<Bid>,
}

#[derive(Debug)]
pub struct Command;

#[derive(Debug)]
pub struct State {
    pub steps: usize,
    pub energy: usize,
    pub harmonics: Harmonics,
    pub matrix: Matrix,
    pub bots: BTreeMap<Bid, Bot>,
    pub trace: Vec<BotCommand>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum WellformedStatus {
    Wellformed,
    NotGroundedWhileLowHarmonics,
    BotInFilledVoxel,
    SeedsAreNotDisjoint,
    SeedIsTheSameAsActiveBot,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Error {
    StateNotWellformed{status: WellformedStatus},
    NotEnoughCommands,
    CommandsInterfere,
    InvalidBid{bid: Bid},
    HaltNotAtZeroCoord,
    HaltTooManyBots,
    HaltNotInLow,
    MoveOutOfBounds {c: Coord},
    MoveRegionIsNotVoid {r: Region},
    NoSeedsAvailable,
    TooBigSplitSeed,
}


impl State {
    pub fn new(matrix: Matrix, trace: Vec<BotCommand>) -> State {
        let mut bots = BTreeMap::new();

        bots.insert(1, Bot {
            pos: Coord { x:0, y:0, z:0 },
            seeds: (2..41).collect(),
        });

        State {
            steps: 0,
            energy: 0,
            harmonics: Harmonics::Low,
            matrix,
            bots,
            trace,
        }
    }

    pub fn wellformed(&self) -> WellformedStatus {
        if let Harmonics::Low = self.harmonics {
            if !self.matrix.all_voxels_are_grounded() {
                return WellformedStatus::NotGroundedWhileLowHarmonics;
            }
        }
        if !self.bots.values().map(|bot| &bot.pos).all(|pos| !self.matrix.is_filled(pos)) {
            return WellformedStatus::BotInFilledVoxel;
        }

        let mut bids_seen = HashSet::new();
        for &bid in self.bots.values().flat_map(|bot| bot.seeds.iter()) {
            if !bids_seen.insert(bid) {
                return WellformedStatus::SeedsAreNotDisjoint;
            }
            if self.bots.contains_key(&bid) {
                return WellformedStatus::SeedIsTheSameAsActiveBot;
            }
        }

        WellformedStatus::Wellformed
    }

    pub fn bot_pos(&self, bid: &Bid) -> Option<Coord> {
        self.bots.get(bid).map(|bot| bot.pos)
    }

    pub fn check_precondition(&self, bid: &Bid, cmd: &BotCommand) -> Result<HashSet<Coord>, Error> {
        if let None = self.bots.get(bid) {
            return Err(Error::InvalidBid{bid:*bid})
        }

        let c = self.bots.get(&bid).unwrap().pos;
        let mut volatile: HashSet<Coord> = [c].iter().cloned().collect();

        match cmd {
            BotCommand::Halt => {
                let check_coord = c.x == 0 && c.y == 0 && c.z == 0;
                let bot_ids: Vec<Bid> = self.bots.keys().cloned().collect();
                let check_the_only_bot = bot_ids == [*bid];
                let check_low = self.harmonics == Harmonics::Low;

                match (check_coord, check_the_only_bot, check_low) {
                    (true, true, true) => (),
                    (false, _, _) => return Err(Error::HaltNotAtZeroCoord),
                    (_, false, _) => return Err(Error::HaltTooManyBots),
                    (_, _, false) => return Err(Error::HaltNotInLow),
                }
            },
            BotCommand::Wait => (),
            BotCommand::Flip => (),
            BotCommand::SMove{ long } => {
                let d = long.to_coord_diff();
                let cf = c.add(d);
                let volatile_reg = Region::from_corners(&c, &cf);

                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                if self.matrix.contains_filled(&volatile_reg) {
                    return Err(Error::MoveRegionIsNotVoid{r: volatile_reg})
                }

                for c in volatile_reg.coord_set().iter() {
                    volatile.insert(*c);
                }
            },
            BotCommand::LMove{ short1, short2 } => {
                let d1 = short1.to_coord_diff();
                let d2 = short2.to_coord_diff();

                let cf = c.add(d1);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }
                let volatile_reg = Region::from_corners(&c, &cf);
                if self.matrix.contains_filled(&volatile_reg) {
                    return Err(Error::MoveRegionIsNotVoid{r: volatile_reg})
                }

                let cff = cf.add(d2);
                if !self.matrix.is_valid_coord(&cff) {
                    return Err(Error::MoveOutOfBounds{c: cff})
                }
                let volatile_reg2 = Region::from_corners(&cf, &cff);
                if self.matrix.contains_filled(&volatile_reg2) {
                    return Err(Error::MoveRegionIsNotVoid{r: volatile_reg2})
                }

                for c in volatile_reg.coord_set().union(&volatile_reg2.coord_set()) {
                    volatile.insert(*c);
                }
            },
            BotCommand::Fill{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                volatile.insert(cf);
            },
            BotCommand::Void{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                volatile.insert(cf);
            },
            BotCommand::Fission{ near, split_m } => {
                let n = *near;
                let cf = c.add(n);
                let bot = self.bots.get(&bid).unwrap();
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }
                if bot.seeds.is_empty() {
                    return Err(Error::NoSeedsAvailable)
                }
                if *split_m as usize > bot.seeds.iter().max().unwrap() + 1 {
                    return Err(Error::TooBigSplitSeed)
                }
                if self.matrix.is_filled(&cf) {
                    return Err(Error::MoveRegionIsNotVoid{r: Region::from_corners(&cf, &cf)})
                }

                volatile.insert(cf);
            }
            BotCommand::FusionP{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }
            },
            BotCommand::FusionS{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }
            },
            BotCommand::GFill{ near: _, far: _ } => unimplemented!(),
            BotCommand::GVoid{ near: _, far: _ } => unimplemented!(),
        }
        Ok(volatile)
    }

    pub fn perform_mut(&mut self, bid: &Bid, cmd: &BotCommand) {
        match cmd {
            BotCommand::Halt => {
                self.bots.remove(&bid);
            },
            BotCommand::Wait => (),
            BotCommand::Flip => {
                match self.harmonics {
                    Harmonics::Low => {
                        self.harmonics = Harmonics::High
                    },
                    Harmonics::High => {
                        self.harmonics = Harmonics::Low
                    },
                };
            },
            BotCommand::SMove{ long } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let d = long.to_coord_diff();
                let cf = c.add(d);

                self.bots.get_mut(&bid).unwrap().pos = cf;
                self.energy += 2 * d.l_1_norm();
            },
            BotCommand::LMove{ short1, short2 } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let d1 = short1.to_coord_diff();
                let d2 = short2.to_coord_diff();

                let cf = c.add(d1);
                let cff = cf.add(d2);

                self.bots.get_mut(&bid).unwrap().pos = cff;
                self.energy += 2 * (d1.l_1_norm() + 2 + d2.l_1_norm());
            },
            BotCommand::Fill{ near } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let n = *near;
                let cf = c.add(n);

                if !self.matrix.is_filled(&cf) {
                    self.matrix.set_filled(&cf);
                    self.energy += 12;
                }
                else {
                    self.energy += 6;
                }
            },
            BotCommand::Void{ near } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let n = *near;
                let cf = c.add(n);

                if self.matrix.is_filled(&cf) {
                    self.matrix.set_void(&cf);
                    self.energy -= 12;
                }
                else {
                    self.energy += 3;
                }
            },
            BotCommand::Fission{ near, split_m } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let n = *near;
                let cf = c.add(n);

                let mut seeds: Vec<Bid> = vec![];
                mem::swap(&mut seeds, &mut self.bots.get_mut(&bid).unwrap().seeds);

                let new_bid = seeds.drain(0..1).next().unwrap();
                let new_seeds = seeds.drain(0..*split_m as usize).collect();

                mem::swap(&mut seeds, &mut self.bots.get_mut(&bid).unwrap().seeds);
                let new_bot = Bot {
                    pos: cf,
                    seeds: new_seeds,
                };
                self.bots.insert(new_bid, new_bot);
                self.energy += 24;
            },
            BotCommand::FusionP{ near } => {
                let c = self.bots.get(&bid).unwrap().pos;

                let n = *near;
                let cf = c.add(n);

                let mut other_bid = *bid;
                let mut seeds: Vec<Bid> = vec![];
                for (bid, secondary) in &self.bots {
                    if secondary.pos == cf {
                        other_bid = *bid;
                        seeds = vec![other_bid];
                        seeds.append(&mut secondary.seeds.to_vec());
                        break;
                    }
                }
                if other_bid != *bid {
                    self.bots.get_mut(&bid).unwrap().seeds.append(&mut seeds);
                    self.bots.get_mut(&bid).unwrap().seeds.sort();
                    self.bots.remove(&other_bid);
                    self.energy -= 24;
                }
            },
            BotCommand::FusionS{ near: _ } => {}, /* Everything is done by FusionP cmd */
            BotCommand::GFill{ near: _, far: _ } => unimplemented!(),
            BotCommand::GVoid{ near: _, far: _ } => unimplemented!(),
        }
    }

    pub fn do_cmd_mut(&mut self, bid: &Bid, cmd: &BotCommand) -> Result<HashSet<Coord>, Error> {
        let res = self.check_precondition(bid, cmd);

        if res.is_ok() {
            self.perform_mut(bid, cmd)
        }

        res
    }

    pub fn step_mut(&mut self, commands: &mut Vec<BotCommand>) -> Result<(), Error> {
        /* check the state is well-formed */
        let wf = self.wellformed();
        if WellformedStatus::Wellformed != wf {
            return Err(Error::StateNotWellformed{status: wf})
        }

        /* check there are enough commands */
        let bids: Vec<Bid> = self.bots.keys().cloned().collect();
        if commands.len() < bids.len() {
            return Err(Error::NotEnoughCommands);
        }

        let commands_to_execute : Vec<BotCommand> = commands.drain(0..bids.len()).collect();

        /* check command preconditions & end commands interference */
        let mut volatile: HashSet<Coord> = HashSet::new();
        for (bid, cmd) in bids.iter().zip(commands_to_execute.iter()) {
            let res = self.check_precondition(bid, &cmd);
            match res {
                Ok(cmd_volatile) => {
                    if volatile.intersection(&cmd_volatile).next() != None {
                        return Err(Error::CommandsInterfere)
                    }
                    for c in cmd_volatile {
                        volatile.insert(c);
                    }
                },
                Err(e) => return Err(e),
            }
        }

        // energy step for the step itself
        match self.harmonics {
            Harmonics::Low =>
                self.energy += 3 * self.matrix.dim() * self.matrix.dim() * self.matrix.dim(),
            Harmonics::High =>
                self.energy += 30 * self.matrix.dim() * self.matrix.dim() * self.matrix.dim(),
        }

        // energy for each nanobot
        self.energy += 20 * self.bots.len();

        for (bid, cmd) in bids.iter().zip(commands_to_execute.iter()) {
            self.perform_mut(bid, &cmd);
        }
        Ok(())
    }

    pub fn run_mut(&mut self, mut commands: Vec<BotCommand>) -> Result<(), Error> {
        // let mut step_counter = 0;
        loop {
            self.steps += 1;
            let res = self.step_mut(&mut commands);
            match res {
                Err(e) => {
                    // println!("ERROR: {:?}", e);
                    // assert!(false);
                    return Err(e);
                },
                Ok(_) => ()
            }

            if commands.is_empty() {
                // println!("ENERGY {} Steps {} ", state.energy, step_counter);
                return Ok(())
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use super::super::{
        coord::{
            Coord,
            CoordDiff,
            Matrix,
            Resolution,
            LinearCoordDiff,
            Axis,
        }
    };

    #[test]
    fn test_init_state() {
        let matrix = Matrix::new(Resolution(10));
        let trace = vec![];
        let st = State::new(matrix, trace.to_vec());

        assert_eq!(st.energy, 0);
        assert_eq!(st.harmonics, Harmonics::Low);
        // assert_eq!(st.matrix, matrix);
        assert_eq!(st.trace, trace);

        // check the first (and the only) bot
        assert_eq!(st.bots.len(), 1);
        let bot = st.bots.get(&1).unwrap();
        assert_eq!(bot.pos, Coord { x:0, y:0, z:0 });
        let seeds : Vec<Bid> = (2..41).collect();
        assert_eq!(bot.seeds, seeds);
    }

    #[test]
    fn do_cmd_halt() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let res = state.do_cmd_mut(&1, &BotCommand::halt().unwrap());
        assert!(res.is_ok());
        assert_eq!(state.bots.len(), 0);
        assert_eq!(state.energy, 0);
        let exp : HashSet<Coord> = [Coord { x: 0, y:0, z: 0, }].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);
        let c = Coord { x: 1, y:0, z: 0, };
        state.bots.get_mut(&1).unwrap().pos = c;
        let res = state.do_cmd_mut(&1, &BotCommand::halt().unwrap());
        assert!(res.is_err());
        assert_eq!(res, Err(Error::HaltNotAtZeroCoord));
        assert_eq!(state.energy, 0);

        /* TODO: Halt too many bots */

        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);
        state.harmonics = Harmonics::High;
        let res = state.do_cmd_mut(&1, &BotCommand::halt().unwrap());
        assert!(res.is_err());
        assert_eq!(res, Err(Error::HaltNotInLow));
        assert_eq!(state.energy, 0);

        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);
        let res = state.do_cmd_mut(&2, &BotCommand::halt().unwrap());
        assert!(res.is_err());
        assert_eq!(state.energy, 0);
        assert_eq!(res, Err(Error::InvalidBid{bid:2}));
    }

    #[test]
    fn do_cmd_wait() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let res = state.do_cmd_mut(&1, &BotCommand::wait().unwrap());
        assert!(res.is_ok());
        assert_eq!(state.energy, 0);
        let exp : HashSet<Coord> = [Coord { x: 0, y:0, z: 0, }].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let res = state.do_cmd_mut(&2, &BotCommand::wait().unwrap());
        assert!(res.is_err());
        assert_eq!(state.energy, 0);
        assert_eq!(res, Err(Error::InvalidBid{bid:2}));
    }

    #[test]
    fn do_cmd_flip() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        // Low -> High
        let res = state.do_cmd_mut(&1, &BotCommand::flip().unwrap());
        assert!(res.is_ok());
        assert_eq!(state.harmonics, Harmonics::High);
        assert_eq!(state.energy, 0);
        let exp : HashSet<Coord> = [Coord { x: 0, y:0, z: 0, }].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        // High -> Low
        let res = state.do_cmd_mut(&1, &BotCommand::flip().unwrap());
        let exp : HashSet<Coord> = [Coord { x: 0, y:0, z: 0, }].iter().cloned().collect();
        assert!(res.is_ok());
        assert_eq!(state.harmonics, Harmonics::Low);
        assert_eq!(state.energy, 0);
        assert_eq!(res.unwrap(), exp);

        let res = state.do_cmd_mut(&2, &BotCommand::flip().unwrap());
        assert!(res.is_err());
        assert_eq!(state.energy, 0);
        assert_eq!(res, Err(Error::InvalidBid{bid:2}));
    }

    #[test]
    fn do_cmd_fill() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        let res = state.do_cmd_mut(&1, &BotCommand::fill(df).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.energy, 12);
        assert!(state.matrix.is_filled(&Coord { x:1, y:0, z:0 }));
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        state.energy = 0; // reset energy
        let res = state.do_cmd_mut(&1, &BotCommand::fill(df).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.energy, 6);
        assert!(state.matrix.is_filled(&Coord { x:1, y:0, z:0 }));
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let matrix = Matrix::new(Resolution(1));
        let mut state = State::new(matrix, vec![]);
        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        let res = state.do_cmd_mut(&1, &BotCommand::fill(df).unwrap());
        assert!(res.is_err());
        assert_eq!(res, Err(Error::MoveOutOfBounds{c: Coord {x:1, y:0, z:0}}));
    }

    #[test]
    fn do_cmd_void() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        /* Intentional BotCommand::fill !!! */
        state.do_cmd_mut(&1, &BotCommand::fill(df).unwrap()).unwrap();

        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        let res = state.do_cmd_mut(&1, &BotCommand::void(df).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.energy, 0);
        assert!(!state.matrix.is_filled(&Coord { x:1, y:0, z:0 }));
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        let res = state.do_cmd_mut(&1, &BotCommand::void(df).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.energy, 3);
        assert!(!state.matrix.is_filled(&Coord { x:1, y:0, z:0 }));
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let matrix = Matrix::new(Resolution(1));
        let mut state = State::new(matrix, vec![]);
        let df = CoordDiff{0: Coord { x:1, y:0, z:0 }};
        let res = state.do_cmd_mut(&1, &BotCommand::void(df).unwrap());
        assert!(res.is_err());
        assert_eq!(res, Err(Error::MoveOutOfBounds{c: Coord {x:1, y:0, z:0}}));
    }

    #[test]
    fn do_cmd_smove() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let lin = LinearCoordDiff::Long { axis: Axis::X, value: 1, };
        let res = state.do_cmd_mut(&1, &BotCommand::smove(lin).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:1, y:0, z:0});
        assert_eq!(state.energy, 2);
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let lin = LinearCoordDiff::Long { axis: Axis::Y, value: 3, };
        state.energy = 0; // reset energy
        let res = state.do_cmd_mut(&1, &BotCommand::smove(lin).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:1, y:3, z:0});
        assert_eq!(state.energy, 6);
        let exp : HashSet<Coord> = [
            Coord { x: 1, y:0, z: 0, },
            Coord { x: 1, y:1, z: 0, },
            Coord { x: 1, y:2, z: 0, },
            Coord { x: 1, y:3, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        // Error cases
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let lin = LinearCoordDiff::Long { axis: Axis::X, value: 4, };
        let res = state.do_cmd_mut(&1, &BotCommand::smove(lin).unwrap());
        assert!(res.is_err());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:0, y:0, z:0});
        assert_eq!(state.energy, 0);
        assert_eq!(res, Err(Error::MoveOutOfBounds{c: Coord { x: 4, y:0, z: 0, }}));

        state.matrix.set_filled(&Coord {x:1, y:0, z:0});
        let lin = LinearCoordDiff::Long { axis: Axis::X, value: 2, };
        let res = state.do_cmd_mut(&1, &BotCommand::smove(lin).unwrap());
        assert!(res.is_err());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:0, y:0, z:0});
        assert_eq!(state.energy, 0);
        assert_eq!(res,Err(Error::MoveRegionIsNotVoid { r: Region {
            min: Coord { x: 0, y: 0, z: 0 },
            max: Coord { x: 2, y: 0, z: 0 },
        }}));
    }


    #[test]
    fn do_cmd_lmove() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let m1 = LinearCoordDiff::Short { axis: Axis::X, value: 1, };
        let m2 = LinearCoordDiff::Short { axis: Axis::Y, value: 1, };
        let res = state.do_cmd_mut(&1, &BotCommand::lmove(m1,m2).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:1, y:1, z:0});
        assert_eq!(state.energy, 8);
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            Coord { x: 1, y:1, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);

        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        let m1 = LinearCoordDiff::Short { axis: Axis::X, value: 2, };
        let m2 = LinearCoordDiff::Short { axis: Axis::Y, value: 2, };
        let res = state.do_cmd_mut(&1, &BotCommand::lmove(m1,m2).unwrap());
        assert!(res.is_ok());
        assert_eq!(state.bot_pos(&1).unwrap(), Coord {x:2, y:2, z:0});
        assert_eq!(state.energy, 12);
        let exp : HashSet<Coord> = [
            Coord { x: 0, y:0, z: 0, },
            Coord { x: 1, y:0, z: 0, },
            Coord { x: 2, y:0, z: 0, },
            Coord { x: 2, y:1, z: 0, },
            Coord { x: 2, y:2, z: 0, },
            ].iter().cloned().collect();
        assert_eq!(res.unwrap(), exp);


        // TODO: Error cases
    }

    #[test]
    fn step_mut() {
        let matrix = Matrix::new(Resolution(4));
        let mut state = State::new(matrix, vec![]);

        // println!("Energy: {}", state.energy);
        let mut trace = vec![
            BotCommand::flip().unwrap(),
            BotCommand::smove(LinearCoordDiff::Long { axis: Axis::X, value: 2, }).unwrap(),
            BotCommand::smove(LinearCoordDiff::Long { axis: Axis::X, value: -2, }).unwrap(),
            BotCommand::flip().unwrap(),
            BotCommand::halt().unwrap(),
            ];
        state.step_mut(&mut trace).unwrap();
        state.step_mut(&mut trace).unwrap();
        state.step_mut(&mut trace).unwrap();
        state.step_mut(&mut trace).unwrap();
        state.step_mut(&mut trace).unwrap();
        // println!("Energy: {}", state.energy);
        // assert!(false);
    }

    use super::super::junk::FA001_TGT_MDL;
    use super::super::junk::FA001_MULTIBOT_NBT;

    #[test]
    fn multibot_fa001() {
        let model = super::super::model::read_model(FA001_TGT_MDL).unwrap().new_empty_of_same_size();
        let matrix = model.new_empty_of_same_size();
        let cmds = super::super::cmd::from_bytes(FA001_MULTIBOT_NBT).unwrap();
        assert_eq!(cmds.len(), 1212);

        let mut state = State::new(matrix, vec![]);
        let res = state.run_mut(cmds);

        assert_eq!(res, Ok(()));
        assert_eq!(state.steps, 212);
        assert_eq!(state.energy, 45727148);
    }
}
