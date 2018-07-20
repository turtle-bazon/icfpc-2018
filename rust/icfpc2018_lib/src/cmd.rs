use coord::{LinearCoordDiff,CoordDiff};
use bit_vec::BitVec;

#[derive(Debug)]
pub enum Error {
    LinearCoordDiffTooLong,
    CoordDiffIsNotNear,
}

#[derive(Debug)]
pub enum BotCommand {
    Halt,
    Wait,
    Flip,
    SMove{ long: LinearCoordDiff },
    LMove{ short1: LinearCoordDiff, short2: LinearCoordDiff },
    Fission{ near: CoordDiff, split_m: u8 },
    Fill{ near: CoordDiff },
    FusionP{ near: CoordDiff },
    FusionS{ near: CoordDiff },
}
impl BotCommand {  
    pub fn halt() -> Result<BotCommand,Error> {
        Ok(BotCommand::Halt)
    }
    pub fn wait() -> Result<BotCommand,Error> {
        Ok(BotCommand::Wait)
    }
    pub fn flip() -> Result<BotCommand,Error> {
        Ok(BotCommand::Flip)
    }
    pub fn smove(mv: LinearCoordDiff) -> Result<BotCommand,Error> {
        match mv {
            LinearCoordDiff::Short { axis, value } | LinearCoordDiff::Long { axis, value } if (value >= -15)&&(value <= 15) => {
                Ok(BotCommand::SMove{ long: LinearCoordDiff::Long { axis: axis, value: value }})
            },
            _  => Err(Error::LinearCoordDiffTooLong),
        }
    }
    pub fn lmove(first: LinearCoordDiff, second: LinearCoordDiff) -> Result<BotCommand,Error> {
        let f = match first {
            LinearCoordDiff::Short { axis, value } | LinearCoordDiff::Long { axis, value } if (value >= -5)&&(value <= 5) => {
                LinearCoordDiff::Short { axis: axis, value: value }
            },
            _  => return Err(Error::LinearCoordDiffTooLong),
        };
        let s = match second {
            LinearCoordDiff::Short { axis, value } | LinearCoordDiff::Long { axis, value } if (value >= -5)&&(value <= 5) => {
                LinearCoordDiff::Short { axis: axis, value: value }
            },
            _  => return Err(Error::LinearCoordDiffTooLong),
        };
        Ok(BotCommand::LMove{ short1: f, short2: s })
    }
    pub fn fission(df: CoordDiff, m: u8) -> Result<BotCommand,Error> {
        if !df.is_near() { return Err(Error::CoordDiffIsNotNear); }
        Ok(BotCommand::Fission{ near: df, split_m: m})
    }
    pub fn fill(df: CoordDiff) -> Result<BotCommand,Error> {
        if !df.is_near() { return Err(Error::CoordDiffIsNotNear); }
        Ok(BotCommand::Fill{ near: df })
    }
    pub fn pfusion(df: CoordDiff) -> Result<BotCommand,Error> {
        if !df.is_near() { return Err(Error::CoordDiffIsNotNear); }
        Ok(BotCommand::FusionP{ near: df })
    }
    pub fn sfusion(df: CoordDiff) -> Result<BotCommand,Error> {
        if !df.is_near() { return Err(Error::CoordDiffIsNotNear); }
        Ok(BotCommand::FusionS{ near: df })
    }
}

pub fn from_bytes(bytes: &[u8]) -> Result<Vec<BotCommands>,Error> {
    unimplemented!()
}

pub fn into_bytes(commands: &Vec<BotCommands>) -> Result<Vec<u8>,Error> {
    unimplemented!()
}
