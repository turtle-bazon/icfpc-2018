extern crate icfpc2018_lib;

use std::{fs, io::{self, Write}};

use icfpc2018_lib::{
    cmd::{
        self,
        BotCommand,
    },
    coord::{
        Axis,
        Coord,
        CoordDiff,
        LinearCoordDiff,
    },
};

fn main() {
    let mut script: Vec<Result<BotCommand, _>> = Vec::new();

    script.extend(
        vec![
            BotCommand::halt(),
            BotCommand::wait(),
            BotCommand::flip(),
        ]
    );
    script.extend(
        (0 .. 10)
            .map(|i| {
                BotCommand::smove(
                    LinearCoordDiff::Long {
                        axis: Axis::X,
                        value: i,
                    },
                )
            })
    );
    script.extend(
        vec![
            BotCommand::lmove(
                LinearCoordDiff::Short {
                    axis: Axis::Y,
                    value: 4,
                },
                LinearCoordDiff::Short {
                    axis: Axis::Z,
                    value: -4,
                },
            ),
            BotCommand::fission(
                CoordDiff(Coord {
                    x: 1,
                    y: 0,
                    z: 0,
                }),
                1,
            ),
            BotCommand::fill(
                CoordDiff(Coord {
                    x: -1,
                    y: 0,
                    z: 0,
                }),
            ),
            BotCommand::pfusion(
                CoordDiff(Coord {
                    x: 0,
                    y: 1,
                    z: 0,
                }),
            ),
            BotCommand::sfusion(
                CoordDiff(Coord {
                    x: 0,
                    y: -1,
                    z: 0,
                }),
            ),
        ]
    );

    let mut validated_script = Vec::with_capacity(script.len());
    for (i, maybe_command) in script.into_iter().enumerate() {
        match maybe_command {
            Ok(cmd) =>
                validated_script.push(cmd),
            Err(err) =>
                panic!("command {} is corrupted: {:?}", i, err),
        }
    }

    let trace = cmd::into_bytes(&validated_script).unwrap();

    let file = fs::File::create("a.nbt").unwrap();
    let mut writer = io::BufWriter::new(file);
    writer.write_all(&trace).unwrap();
}
