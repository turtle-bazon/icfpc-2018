use std::collections::{HashSet, BTreeMap};
use std::mem;

use super::{
    coord::{
        Axis,
        LinearCoordDiff,
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

    pub fn is_halt(&self) -> bool {
        self.bots.is_empty()
    }

    pub fn check_precondition(&self, bid: &Bid, cmd: &BotCommand) -> Result<(Region, Option<Region>), Error> {
        if let None = self.bots.get(bid) {
            return Err(Error::InvalidBid{bid:*bid})
        }

        let c = self.bot_pos(&bid).unwrap();
        let bot_reg = Region::from_corners(&c,&c);

        match cmd {
            BotCommand::Halt => {
                let check_coord = c.x == 0 && c.y == 0 && c.z == 0;
                let bot_ids: Vec<Bid> = self.bots.keys().cloned().collect();
                let check_the_only_bot = bot_ids == [*bid];
                let check_low = self.harmonics == Harmonics::Low;

                match (check_coord, check_the_only_bot, check_low) {
                    (true, true, true) => Ok((bot_reg, None)),
                    (false, _, _) => return Err(Error::HaltNotAtZeroCoord),
                    (_, false, _) => return Err(Error::HaltTooManyBots),
                    (_, _, false) => return Err(Error::HaltNotInLow),
                }
            },
            BotCommand::Wait => Ok((bot_reg, None)),
            BotCommand::Flip => Ok((bot_reg, None)),
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

                Ok((volatile_reg, None))
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

                let volatile_reg2 = match short2 {
                    LinearCoordDiff::Short{axis,value} =>
                        match axis {
                            Axis::X => {
                                if *value > 0 {
                                    Region::from_corners(&Coord { x: cf.x + 1, y: cf.y, z: cf.z, }, &cff)
                                }
                                else {
                                    Region::from_corners(&Coord { x: cf.x - 1, y: cf.y, z: cf.z, }, &cff)
                                }
                            },
                            Axis::Y => {
                                if *value > 0 {
                                    Region::from_corners(&Coord { x: cf.x, y: cf.y + 1, z: cf.z, }, &cff)
                                }
                                else {
                                    Region::from_corners(&Coord { x: cf.x, y: cf.y - 1, z: cf.z, }, &cff)
                                }
                            }
                            Axis::Z => {
                                if *value > 0 {
                                    Region::from_corners(&Coord { x: cf.x, y: cf.y, z: cf.z + 1, }, &cff)
                                }
                                else {
                                    Region::from_corners(&Coord { x: cf.x, y: cf.y, z: cf.z - 1, }, &cff)
                                }
                            }
                        },
                    _ => panic!("Unexpected diff!"),
                };
                if self.matrix.contains_filled(&volatile_reg2) {
                    return Err(Error::MoveRegionIsNotVoid{r: volatile_reg2})
                }

                Ok((volatile_reg, Some(volatile_reg2)))
            },
            BotCommand::Fill{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                Ok((Region::from_corners(&c, &cf), None))
            },
            BotCommand::Void{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                Ok((Region::from_corners(&c, &cf), None))
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

                Ok((Region::from_corners(&c, &cf), None))
            }
            BotCommand::FusionP{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                Ok((bot_reg, None))
            },
            BotCommand::FusionS{ near } => {
                let n = *near;
                let cf = c.add(n);
                if !self.matrix.is_valid_coord(&cf) {
                    return Err(Error::MoveOutOfBounds{c: cf})
                }

                Ok((bot_reg, None))
            },
            BotCommand::GFill{ near: _, far: _ } => unimplemented!(),
            BotCommand::GVoid{ near: _, far: _ } => unimplemented!(),
        }
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

        let mut ret = HashSet::new();

        match res {
            Ok((vol1, maybe_vol2)) => {
                for c in vol1.coord_set() {
                    ret.insert(c);
                }

                if let Some(vol2) = maybe_vol2 {
                    for c in vol2.coord_set() {
                        ret.insert(c);
                    }
                }

                Ok(ret)
            },
            Err(e) => Err(e),
        }
    }

    pub fn step_mut<T>(&mut self, cmd_iter: &mut T) -> Result<(), Error>
        where T : Iterator<Item = BotCommand> {

        /* check the state is well-formed */
        let wf = self.wellformed();
        if WellformedStatus::Wellformed != wf {
            return Err(Error::StateNotWellformed{status: wf})
        }

        /* check there are enough commands */
        let bids: Vec<Bid> = self.bots.keys().cloned().collect();
        let cmds: Vec<BotCommand> = cmd_iter.take(bids.len()).collect();

        /* check command preconditions & end commands interference */
        let mut volatile: Vec<Region> = vec![];
        let mut bid_iter = bids.iter();
        let mut cmd_iter = cmds.iter();
        loop {
            match (bid_iter.next(), cmd_iter.next()) {
                (Some(bid), Some(cmd)) => {
                    let res = self.check_precondition(bid, &cmd);
                    match res {
                        Ok((vol1, maybe_vol2)) => {
                            for vol_reg in &volatile {
                                if vol_reg.intersects(&vol1) {
                                    println!("cmd: {:?}", cmd);
                                    return Err(Error::CommandsInterfere)
                                }
                            }
                            volatile.push(vol1);

                            if let Some(vol2) = maybe_vol2 {
                                for vol_reg in &volatile {
                                    if vol_reg.intersects(&vol2) {
                                        println!("cmd: {:?}", cmd);
                                        return Err(Error::CommandsInterfere)
                                    }
                                }

                                volatile.push(vol2);
                            }
                        },
                        Err(e) => return Err(e),
                    }
                },
                (Some(_), None) => { return Err(Error::NotEnoughCommands); }
                (None, _) => { break; }
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

        let mut bid_iter = bids.iter();
        let mut cmd_iter = cmds.iter();
        loop {
            match (bid_iter.next(), cmd_iter.next()) {
                (Some(bid), Some(cmd)) => {
                    self.perform_mut(bid, &cmd);
                },
                (Some(_), None) => { return Err(Error::NotEnoughCommands); }
                (None, _) => { break; }
            }
        }
        Ok(())
    }

    pub fn run_mut(&mut self, commands: Vec<BotCommand>) -> Result<(), Error> {
        let mut cmd_iter = commands.into_iter();
        loop {
            self.steps += 1;
            let res = self.step_mut(&mut cmd_iter);
            match res {
                Err(e) => {
                    return Err(e);
                },
                Ok(_) => ()
            }

            if self.is_halt() {
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
        let mut trace_it = trace.into_iter();
        state.step_mut(&mut trace_it).unwrap();
        state.step_mut(&mut trace_it).unwrap();
        state.step_mut(&mut trace_it).unwrap();
        state.step_mut(&mut trace_it).unwrap();
        state.step_mut(&mut trace_it).unwrap();
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
