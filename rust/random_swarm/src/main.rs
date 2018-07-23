extern crate rand;
extern crate env_logger;
extern crate icfpc2018_lib;
#[macro_use] extern crate log;
#[macro_use] extern crate clap;

use std::{io::{self, Write}, fs, process};
use clap::Arg;
use rand::{SeedableRng, prng::XorShiftRng};

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
    MissingParameter(&'static str),
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
             .default_value("../../problems/FA004_tgt.mdl")
             .takes_value(true))
        .get_matches();

    let target_model_file = matches.value_of("target-model")
        .ok_or(Error::MissingParameter("target-model"))?;
    let target_model = model::read_model_file(target_model_file)
            .map_err(Error::Model)?;
    let source_model = if let Some(source_model_file) = matches.value_of("souce-model") {
        model::read_model_file(source_model_file)
            .map_err(Error::Model)?
    } else {
        Matrix::new(Resolution(target_model.dim() as M))
    };

    let mut rng: XorShiftRng =
        SeedableRng::from_seed([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

    let solve_result = random_swarm::solve_rng(
        source_model,
        target_model,
        random_swarm::Config {
            init_bots: vec![],
            rtt_limit: 64,
            route_attempts_limit: 16,
            global_ticks_limit: 100,
        },
        &mut rng,
    );

    let script = match solve_result {
        Ok(script) =>
            script,
        Err(random_swarm::Error::GlobalTicksLimitExceeded { ticks, script_so_far, }) => {
            warn!("global ticks limit exceeded ({}), saving script of {} commands", ticks, script_so_far.len());
            script_so_far
        },
        Err(error) =>
            return Err(Error::Solver(error)),
    };

    let trace = cmd::into_bytes(&script)
        .map_err(Error::OutScriptFileCompile)?;
    let file = fs::File::create("a.nbt")
        .map_err(Error::OutScriptFileOpen)?;
    let mut writer = io::BufWriter::new(file);
    writer.write_all(&trace)
        .map_err(Error::OutScriptFileWrite)?;

    Ok(())
}
