use coord::{LinearCoordDiff,CoordDiff};

#[derive(Debug)]
pub enum Error {
    LinearCoordDiffTooLong,
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
    pub fn smove(mv: LinearCoordDiff) -> Result<BotCommand,Error> {
        match mv {
            LinearCoordDiff::Short { axis, value } | LinearCoordDiff::Long { axis, value } if (value >= -15)&&(value <= 15) => {
                Ok(BotCommand::SMove{ long: LinearCoordDiff::Long { axis: axis, value: value }})
            },
            _  => Err(Error::LinearCoordDiffTooLong),
        }
    }
}
