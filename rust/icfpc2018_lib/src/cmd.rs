use coord::{LinearCoordDiff,CoordDiff}; 

#[derive(Debug)]
pub enum Error {
    LinearCoordDiffTooLong,
}

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
}
