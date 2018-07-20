use coord::{LinearCoordDiff,CoordDiff,Coord,Axis};


#[derive(Debug)]
pub enum Error {
    LinearCoordDiffTooLong,
    CoordDiffIsNotNear,
    DeserializeNotNear(u8),
    DeserializeNotAxis(u8),
    DeserializeUnknown(u8),
    DeserializeSMoveDiff(u8),
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
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


pub fn check_near(b: u8) -> Result<CoordDiff,Error> {
    let z = b % 3;
    let y = (b % 9) / 3;
    let x = b / 9;
    if (x<3)&&(y<3)&&(z<3) {
        Ok(CoordDiff(Coord{
            x: (x as isize) - 1,
            y: (y as isize) - 1,
            z: (z as isize) - 1,
        }))
    } else {
        Err(Error::DeserializeNotNear(b))
    }
}

pub fn check_axis(b: u8) -> Result<Axis,Error> {
    match b {
        0b01 => Ok(Axis::X),
        0b10 => Ok(Axis::Y),
        0b11 => Ok(Axis::Z),
        _ => Err(Error::DeserializeNotAxis(b))
    }
}

pub fn from_bytes(bytes: &[u8]) -> Result<Vec<BotCommand>,Error> {
    let mut i = 0;
    let len = bytes.len();
    let mut res = Vec::new();
    while i<len {
        let b = bytes[i];
        match ((b>>3) & 0b11111, b & 0b111) {
            (0b11111,0b111) => res.push(BotCommand::halt()?), 
            (0b11111,0b110) => res.push(BotCommand::wait()?), 
            (0b11111,0b101) => res.push(BotCommand::flip()?),
            (near,0b011) => res.push(BotCommand::fill(check_near(near)?)?),
            (near,0b111) => res.push(BotCommand::pfusion(check_near(near)?)?),
            (near,0b110) => res.push(BotCommand::sfusion(check_near(near)?)?),
            (near,0b101) => {
                let df = check_near(near)?;
                i += 1;
                let m = bytes[i];
                res.push(BotCommand::fission(df,m)?)
            },
            (p,0b100) => {
                match ((p>>3) & 0b11,(p>>1) & 0b11, p & 0b1) {
                    (0,axis,0) => {
                        let ax = check_axis(axis)?;
                        i += 1;
                        let d = bytes[i];
                        if (d & 0b11100000) > 0 { return Err(Error::DeserializeSMoveDiff(d)); }
                        res.push(BotCommand::smove(LinearCoordDiff::Long{
                            axis: ax,
                            value: ((d & 0b11111) as isize) - 15,
                        })?)
                    },
                    (axis2,axis1,1) => {
                        let ax1 = check_axis(axis1)?;
                        let ax2 = check_axis(axis2)?;
                        i += 1;
                        let d = bytes[i];
                        res.push(BotCommand::lmove(
                            LinearCoordDiff::Short{
                                axis: ax1,
                                value: ((d & 0b1111) as isize) - 5,
                            },
                            LinearCoordDiff::Short{
                                axis: ax2,
                                value: (((d >> 4) & 0b11111) as isize) - 5,
                            }
                            )?)
                    },
                    (_,_,_) => return Err(Error::DeserializeUnknown(b)),
                }
            },
            (_,_) => return Err(Error::DeserializeUnknown(b)),
        }
        i += 1;
    }
    Ok(res)
}

pub fn into_bytes(commands: &Vec<BotCommand>) -> Result<Vec<u8>,Error> {
    unimplemented!()
}


#[cfg(test)]
mod test {
    use super::*;
    
    #[test]
    fn test_deser_halt() {
        let buf = [0b11111111];
        let res = vec![BotCommand::halt().unwrap()];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    #[test]
    fn test_deser_wait() {
        let buf = [0b11111110];
        let res = vec![BotCommand::wait().unwrap()];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    #[test]
    fn test_deser_flip() {
        let buf = [0b11111101];
        let res = vec![BotCommand::flip().unwrap()];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, SMove <12,0,0> is encoded as [00010100] [00011011] and SMove <0,0,-4> is encoded as [00110100] [00001011].
    #[test]
    fn test_deser_smove() {
        let buf = [0b00010100,0b00011011,0b00110100,0b00001011];
        let res = vec![
            BotCommand::smove(LinearCoordDiff::Long{
                axis: Axis::X,
                value: 12,
            }).unwrap(),
            BotCommand::smove(LinearCoordDiff::Long{
                axis: Axis::Z,
                value: -4,
            }).unwrap(),
                       ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, LMove <3,0,0> <0,-5,0> is encoded as [10011100] [00001000] and LMove <0,-2,0> <0,0,2> is encoded as [11101100] [01110011].
    #[test]
    fn test_deser_lmove() {
        let buf = [0b10011100,0b00001000,0b11101100,0b01110011];
        let res = vec![
            BotCommand::lmove(
                LinearCoordDiff::Short{
                    axis: Axis::X,
                    value: 3,
                },
                LinearCoordDiff::Short{
                    axis: Axis::Y,
                    value: -5,
                }).unwrap(),
            BotCommand::lmove(
                LinearCoordDiff::Short{
                    axis: Axis::Y,
                    value: -2,
                },
                LinearCoordDiff::Short{
                    axis: Axis::Z,
                    value: 2,
                }).unwrap(),
                       ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, FusionP <-1,1,0> is encoded as [00111111].
    #[test]
    fn test_deser_fus_p() {
        let buf = [0b00111111];
        let res = vec![
            BotCommand::pfusion(CoordDiff(Coord {
                x: -1,
                y: 1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, FusionS <1,-1,0> is encoded as [10011110].
    #[test]
    fn test_deser_fus_s() {
        let buf = [0b10011110];
        let res = vec![
            BotCommand::sfusion(CoordDiff(Coord {
                x: 1,
                y: -1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, Fission <0,0,1> 5 is encoded as [01110101] [00000101].
    #[test]
    fn test_deser_fission() {
        let buf = [0b01110101,0b00000101];
        let res = vec![
            BotCommand::fission(CoordDiff(Coord {
                x: 0,
                y: 0,
                z: 1,
            }),5).unwrap()
                ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }

    // For example, Fill <0,-1,0> is encoded as [01010011].
    #[test]
    fn test_deser_fill() {
        let buf = [0b01010011];
        let res = vec![
            BotCommand::fill(CoordDiff(Coord {
                x: 0,
                y: -1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }
}
