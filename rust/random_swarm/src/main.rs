extern crate rand;
extern crate env_logger;
extern crate icfpc2018_lib;
#[macro_use] extern crate log;
#[macro_use] extern crate clap;

use std::{io::{self, Write}, fs, process};
use clap::Arg;

use icfpc2018_lib::{
    coord::{
        M,
        Matrix,
        Resolution,
    },
    cmd,
    model,
    solver::random_swarm,
};

fn main() {
    env_logger::init();
    match run() {
        Ok(()) =>
            info!("graceful shutdown"),
        Err(e) => {
            error!("fatal error: {:?}", e);
            process::exit(1);
        },
    }
}

#[derive(Debug)]
enum Error {
    InvalidIntegerValue(clap::Error),
    NoSourceOrTargetModelProvided,
    Model(model::Error),
    Solver(random_swarm::Error),
    OutScriptFileCompile(cmd::Error),
    OutScriptFileOpen(io::Error),
    OutScriptFileWrite(io::Error),
}

fn run() -> Result<(), Error> {
    let matches = app_from_crate!()
        .arg(Arg::with_name("source-model")
             .short("s")
             .long("source-model")
             .value_name("FILE")
             .help("Source model")
             .takes_value(true))
        .arg(Arg::with_name("target-model")
             .short("t")
             .long("target-model")
             .value_name("FILE")
             .help("Target model")
             .takes_value(true))
        .arg(Arg::with_name("global-ticks-limit")
             .short("l")
             .long("global-ticks-limit")
             .value_name("LIMIT")
             .help("Solver global ticks limit parameter")
             .default_value("1024")
             .takes_value(true))
        .arg(Arg::with_name("rtt-limit")
             .short("r")
             .long("rtt-limit")
             .value_name("LIMIT")
             .help("Solver RTT router samples limit")
             .default_value("256")
             .takes_value(true))
        .arg(Arg::with_name("route-attempts-limit")
             .long("route-attempts-limit")
             .value_name("LIMIT")
             .help("Solver RTT router attempts limit")
             .default_value("16")
             .takes_value(true))
        .arg(Arg::with_name("max-spawns")
             .short("M")
             .long("max-spawns")
             .value_name("COUNT")
             .help("Solver maximum child spawns limit")
             .default_value("1")
             .takes_value(true))
        .arg(Arg::with_name("output")
             .short("o")
             .long("output")
             .value_name("FILE")
             .help("Trace file output")
             .default_value("a.nbt")
             .takes_value(true))
        .get_matches();
    let (source_model, target_model) =
        if let Some(source_model_file) = matches.value_of("source-model") {
            if let Some(target_model_file) = matches.value_of("target-model") {
                (model::read_model_file(source_model_file).map_err(Error::Model)?,
                 model::read_model_file(target_model_file).map_err(Error::Model)?)
            } else {
                let source_model = model::read_model_file(source_model_file).map_err(Error::Model)?;
                let target_model = Matrix::new(Resolution(source_model.dim() as M));
                (source_model, target_model)
            }
        } else {
            if let Some(target_model_file) = matches.value_of("target-model") {
                let target_model = model::read_model_file(target_model_file).map_err(Error::Model)?;
                let source_model = Matrix::new(Resolution(target_model.dim() as M));
                (source_model, target_model)
            } else {
                return Err(Error::NoSourceOrTargetModelProvided);
            }
        };

    info!("source model with {} voxels", source_model.filled_voxels().count());
    info!("target model with {} voxels", target_model.filled_voxels().count());

    let config = random_swarm::Config {
        init_bots: vec![],
        rtt_limit: value_t!(matches, "rtt-limit", usize)
            .map_err(Error::InvalidIntegerValue)?,
        route_attempts_limit: value_t!(matches, "route-attempts-limit", usize)
            .map_err(Error::InvalidIntegerValue)?,
        global_ticks_limit: value_t!(matches, "global-ticks-limit", usize)
            .map_err(Error::InvalidIntegerValue)?,
        max_spawns: value_t!(matches, "max-spawns", usize)
            .map_err(Error::InvalidIntegerValue)?,
    };

    info!("Everything is ready, start solving");

    let solve_result = random_swarm::solve_rng(
        source_model,
        target_model,
        config,
        &mut rand::thread_rng(),
    );

    let (script, status) = match solve_result {
        Ok(script) =>
            (script, Ok(())),
        Err((error, script)) =>
            (script, Err(Error::Solver(error))),
    };
    info!("saving script of {} commands", script.len());

    let trace = cmd::into_bytes(&script)
        .map_err(Error::OutScriptFileCompile)?;
    let output_filename = value_t!(matches, "output", String).unwrap();
    let file = fs::File::create(output_filename)
        .map_err(Error::OutScriptFileOpen)?;
    let mut writer = io::BufWriter::new(file);
    writer.write_all(&trace)
        .map_err(Error::OutScriptFileWrite)?;
    status
}
