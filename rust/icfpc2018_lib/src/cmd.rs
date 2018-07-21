use coord::{LinearCoordDiff,CoordDiff,Coord,Axis};


#[derive(Debug)]
pub enum Error {
    RestrictedLinearCoordDiff,
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
            LinearCoordDiff::Long { value, .. } if (value >= -15)&&(value <= 15) => {
                Ok(BotCommand::SMove{ long: mv })
            },
            LinearCoordDiff::Long { .. }  => Err(Error::LinearCoordDiffTooLong),
            LinearCoordDiff::Short { .. }  => Err(Error::RestrictedLinearCoordDiff),
        }
    }
    pub fn lmove(first: LinearCoordDiff, second: LinearCoordDiff) -> Result<BotCommand,Error> {
        let f = match first {
            LinearCoordDiff::Short { value, .. } if (value >= -5)&&(value <= 5) => first,
            LinearCoordDiff::Short { .. }  => return Err(Error::LinearCoordDiffTooLong),
            LinearCoordDiff::Long { .. } => return Err(Error::RestrictedLinearCoordDiff),
        };
        let s = match second {
            LinearCoordDiff::Short { value, .. } if (value >= -5)&&(value <= 5) => second,
            LinearCoordDiff::Short { .. }  => return Err(Error::LinearCoordDiffTooLong),
            LinearCoordDiff::Long { .. } => return Err(Error::RestrictedLinearCoordDiff),
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

fn near_to_u8(df: &CoordDiff) -> Result<u8,Error> {
    if !df.is_near() { return Err(Error::CoordDiffIsNotNear); }
    Ok(((df.0.x + 1) as u8)*9 + ((df.0.y + 1) as u8)*3 + ((df.0.z + 1) as u8))
}

fn check_near(b: u8) -> Result<CoordDiff,Error> {
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

fn axis_to_u8(ax: &Axis) -> Result<u8,Error> {
    match ax {
        Axis::X => Ok(0b01),
        Axis::Y => Ok(0b10),
        Axis::Z => Ok(0b11),
    }
}

fn check_axis(b: u8) -> Result<Axis,Error> {
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
    let mut res = Vec::new();
    for c in commands {
        match c {
            BotCommand::Halt => res.push(0b11111111),
            BotCommand::Wait => res.push(0b11111110),
            BotCommand::Flip => res.push(0b11111101),
            BotCommand::Fill{ near } => res.push( ((near_to_u8(near)? & 0b11111)<<3) | 0b011),
            BotCommand::FusionP{ near } => res.push( ((near_to_u8(near)? & 0b11111)<<3) | 0b111),
            BotCommand::FusionS{ near } => res.push( ((near_to_u8(near)? & 0b11111)<<3) | 0b110),
            BotCommand::Fission{ near , split_m } => {
                res.push( ((near_to_u8(near)? & 0b11111)<<3) | 0b101);
                res.push(*split_m)
            },
            BotCommand::SMove{ long } => {
                match long {
                    LinearCoordDiff::Long { axis, value } if (*value >= -15)&&(*value <= 15) => {
                        res.push( ((axis_to_u8(axis)? & 0b11)<<4) | 0b0100);
                        res.push( ((value + 15) as u8) & 0b00011111 )
                    },
                    LinearCoordDiff::Long { .. }  => return Err(Error::LinearCoordDiffTooLong),
                    LinearCoordDiff::Short { .. }  => return Err(Error::RestrictedLinearCoordDiff),
                }
            },
            BotCommand::LMove{ ref short1, ref short2 } => {
                let (ax1, val1) = match short1 {
                    LinearCoordDiff::Short { axis, value } if (*value >= -5)&&(*value <= 5) => {
                         (axis_to_u8(axis)?, (value+5) as u8)
                    },
                    LinearCoordDiff::Long { .. }  => return Err(Error::RestrictedLinearCoordDiff),
                    LinearCoordDiff::Short { .. }  => return Err(Error::LinearCoordDiffTooLong),
                };
                let (ax2, val2) = match short2 {
                    LinearCoordDiff::Short { axis, value } if (*value >= -5)&&(*value <= 5) => {
                         (axis_to_u8(axis)?, (value+5) as u8)
                    },
                    LinearCoordDiff::Long { .. }  => return Err(Error::RestrictedLinearCoordDiff),
                    LinearCoordDiff::Short { .. }  => return Err(Error::LinearCoordDiffTooLong),
                };
                res.push( ((ax2 & 0b11) << 6) |
                          ((ax1 & 0b11) << 4) | 0b1100 );
                res.push( ((val2 & 0b1111) << 4) | (val1 & 0b1111) )
            }            
        }
    }
    Ok(res)
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
    fn test_ser_halt() {
        let buf = vec![0b11111111];
        let res = vec![BotCommand::halt().unwrap()];
        assert_eq!(buf,into_bytes(&res).unwrap());
    }

    #[test]
    fn test_deser_wait() {
        let buf = [0b11111110];
        let res = vec![BotCommand::wait().unwrap()];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }
    #[test]
    fn test_ser_wait() {
        let buf = vec![0b11111110];
        let res = vec![BotCommand::wait().unwrap()];
        assert_eq!(buf,into_bytes(&res).unwrap());
    }

    #[test]
    fn test_deser_flip() {
        let buf = [0b11111101];
        let res = vec![BotCommand::flip().unwrap()];
        assert_eq!(res,from_bytes(&buf).unwrap());
    }
    #[test]
    fn test_ser_flip() {
        let buf = vec![0b11111101];
        let res = vec![BotCommand::flip().unwrap()];
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_smove() {
        let buf = vec![0b00010100,0b00011011,0b00110100,0b00001011];
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
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_lmove() {
        let buf = vec![0b10011100,0b00001000,0b11101100,0b01110011];
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
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_fus_p() {
        let buf = vec![0b00111111];
        let res = vec![
            BotCommand::pfusion(CoordDiff(Coord {
                x: -1,
                y: 1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_fus_s() {
        let buf = vec![0b10011110];
        let res = vec![
            BotCommand::sfusion(CoordDiff(Coord {
                x: 1,
                y: -1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_fission() {
        let buf = vec![0b01110101,0b00000101];
        let res = vec![
            BotCommand::fission(CoordDiff(Coord {
                x: 0,
                y: 0,
                z: 1,
            }),5).unwrap()
                ];
        assert_eq!(buf,into_bytes(&res).unwrap());
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
    #[test]
    fn test_ser_fill() {
        let buf = vec![0b01010011];
        let res = vec![
            BotCommand::fill(CoordDiff(Coord {
                x: 0,
                y: -1,
                z: 0,
            })).unwrap()
                ];
        assert_eq!(buf,into_bytes(&res).unwrap());
    }

    #[test]
    fn test_la003_nbt() {
        let buf = vec![
            0xfe, 0x34, 0x10, 0x24, 0x10, 0x14, 0x10, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34,
            0x10, 0x34, 0x10, 0x14, 0x10, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34,
            0x0e, 0x34, 0x0e, 0x14, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x34, 0x10, 0x14, 0x10, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x14, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x14, 0x10, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x14, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x14, 0x10, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x14, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34,
            0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10,
            0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x14, 0x10, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x14, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x14, 0x10, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x14, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x14, 0x10, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x14, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x14, 0x10, 0x34, 0x0e, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53,
            0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34, 0x0e, 0x14, 0x10, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x53,
            0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34,
            0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10, 0x53, 0x34, 0x10,
            0x53, 0x34, 0x10, 0x34, 0x10, 0x34, 0x10, 0x14, 0x10, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x34,
            0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e,
            0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x53, 0x34, 0x0e, 0x34,
            0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x34, 0x0e, 0x24, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14,
            0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14,
            0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0x14, 0x0e, 0xfe, 0xff
                ];
        let cmds = from_bytes(&buf).unwrap();
        let buf2 = into_bytes(&cmds).unwrap();
        assert_eq!(buf,buf2);
    }
}
