extern crate gfx_core;
extern crate env_logger;
extern crate piston_window;
extern crate gfx_debug_draw;
extern crate gfx_text;
extern crate camera_controllers;
extern crate vecmath;
extern crate icfpc2018_lib;
#[macro_use] extern crate gfx;
#[macro_use] extern crate log;
#[macro_use] extern crate clap;

use std::{
    process,
};

use clap::Arg;
use piston_window::{
    OpenGL,
    PistonWindow,
    WindowSettings,
    RenderEvent,
    ResizeEvent,
    Event,
    Input,
    Button,
    ButtonArgs,
    ButtonState,
    // MouseButton,
    // Motion,
    Key,
};
use gfx_debug_draw::DebugRenderer;
use vecmath::{mat4_id, vec3_add};
use camera_controllers::{
    OrbitZoomCamera,
    OrbitZoomCameraSettings,
    CameraPerspective,
    model_view_projection
};

use icfpc2018_lib::model;

mod voxel;

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
    Piston(PistonError),
}

#[derive(Debug)]
enum PistonError {
    BuildWindow(String),
    // LoadFont { file: String, error: io::Error, },
    // DrawText(gfx_core::factory::CombinedError),
    DebugRendererInit(gfx_debug_draw::DebugRendererError),
    DebugRendererRender(gfx_debug_draw::DebugRendererError),
    VoxelRenderer(voxel::Error),
}

const SCREEN_WIDTH: u32 = 640;
const SCREEN_HEIGHT: u32 = 480;

fn run() -> Result<(), Error> {
    let matches = app_from_crate!()
        .arg(Arg::with_name("assets-dir")
             .short("a")
             .long("assets-dir")
             .value_name("DIR")
             .help("Graphics resources directory")
             .default_value("./assets")
             .takes_value(true))
        .arg(Arg::with_name("model")
             .short("m")
             .long("model")
             .value_name("FILE")
             .help("Model file to visualize")
             .default_value("../../problems/LA001_tgt.mdl")
             .takes_value(true))
        .get_matches();

    let _assets_dir = matches.value_of("assets-dir")
        .ok_or(Error::MissingParameter("assets-dir"))?;
    let model_file = matches.value_of("model")
        .ok_or(Error::MissingParameter("model"))?;

    let matrix = model::read_model_file(model_file)
        .map_err(Error::Model)?;

    let opengl = OpenGL::V4_1;
    let mut window: PistonWindow = WindowSettings::new("icfpc2018 visualizer", [SCREEN_WIDTH, SCREEN_HEIGHT])
        .exit_on_esc(true)
        .opengl(opengl)
        .build()
        .map_err(PistonError::BuildWindow)
        .map_err(Error::Piston)?;

    let mut debug_renderer = {
        let text_renderer = {
            gfx_text::new(window.factory.clone()).unwrap()
        };
        DebugRenderer::new(window.factory.clone(), text_renderer, 64)
            .map_err(PistonError::DebugRendererInit)
            .map_err(Error::Piston)?
    };
    let mut voxel_renderer = voxel::VoxelRenderer::new(&mut window.factory, 64)
        .map_err(PistonError::VoxelRenderer)
        .map_err(Error::Piston)?;

    let model = mat4_id();
    let mut projection = CameraPerspective {
        fov: 90.0f32,
        near_clip: 0.1,
        far_clip: 1000.0,
        aspect_ratio: (SCREEN_WIDTH as f32) / (SCREEN_HEIGHT as f32)
    }.projection();

    let mut orbit_zoom_camera: OrbitZoomCamera<f32> = OrbitZoomCamera::new(
        [0.0, 0.0, 0.0],
        OrbitZoomCameraSettings::default()
    );

    // let mut font_path = PathBuf::from(assets_dir);
    // font_path.push("FiraSans-Regular.ttf");
    // let mut glyphs = Glyphs::new(&font_path, window.factory.clone(), TextureSettings::new())
    //     .map_err(|e| Error::Piston(PistonError::LoadFont {
    //         file: font_path.to_string_lossy().to_string(),
    //         error: e,
    //     }))?;

    loop {
        let event = if let Some(ev) = window.next() {
            ev
        } else {
            return Ok(());
        };

        orbit_zoom_camera.event(&event);
        event.resize(|width, height| {
            // Update projection matrix
            projection = CameraPerspective {
                fov: 90.0f32,
                near_clip: 0.1,
                far_clip: 1000.0,
                aspect_ratio: (width as f32) / (height as f32)
            }.projection();
        });

        let maybe_result = window.draw_3d(&event, |win| {
            if let Some(args) = event.render_args() {
                win.encoder.clear(&win.output_color, [0.3, 0.3, 0.3, 1.0]);
                win.encoder.clear_depth(&win.output_stencil, 1.0);

                let camera_view = orbit_zoom_camera.camera(args.ext_dt).orthogonal();

                let camera_projection = model_view_projection(
                    model,
                    camera_view,
                    projection,
                );

                // Draw axes
                debug_renderer.draw_line([0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0]);
                debug_renderer.draw_line([0.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 1.0, 0.0, 1.0]);
                debug_renderer.draw_line([0.0, 0.0, 0.0], [0.0, 0.0, 5.0], [0.0, 0.0, 1.0, 1.0]);

                debug_renderer.draw_text_at_position(
                    "X",
                    [6.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 1.0],
                );

                debug_renderer.draw_text_at_position(
                    "Y",
                    [0.0, 6.0, 0.0],
                    [0.0, 1.0, 0.0, 1.0],
                );

                debug_renderer.draw_text_at_position(
                    "Z",
                    [0.0, 0.0, 6.0],
                    [0.0, 0.0, 1.0, 1.0],
                );

                // Draw matrix
                for voxel in matrix.filled_voxels() {
                    // let min_point = [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                    // let max_point = vec3_add(min_point, [1.0, 1.0, 1.0]);
                    // voxel_renderer.draw_voxel(min_point, max_point, [0.0, 0.0, 0.0, 1.0]);

                    let position =
                        [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                    let mut draw_edge = |diff_src, diff_dst| {
                        debug_renderer.draw_line(
                            vec3_add(position, diff_src),
                            vec3_add(position, diff_dst),
                            [0.0, 0.0, 0.0, 1.0],
                        );
                    };
                    draw_edge([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
                    draw_edge([1.0, 0.0, 0.0], [1.0, 1.0, 0.0]);
                    draw_edge([1.0, 1.0, 0.0], [0.0, 1.0, 0.0]);
                    draw_edge([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]);
                    draw_edge([0.0, 0.0, 1.0], [1.0, 0.0, 1.0]);
                    draw_edge([1.0, 0.0, 1.0], [1.0, 1.0, 1.0]);
                    draw_edge([1.0, 1.0, 1.0], [0.0, 1.0, 1.0]);
                    draw_edge([0.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
                    draw_edge([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
                    draw_edge([1.0, 0.0, 0.0], [1.0, 0.0, 1.0]);
                    draw_edge([1.0, 1.0, 0.0], [1.0, 1.0, 1.0]);
                    draw_edge([0.0, 1.0, 0.0], [0.0, 1.0, 1.0]);
                }

                voxel_renderer.render(&mut win.encoder, &mut win.factory, &win.output_color, &win.output_stencil, camera_projection)
                    .map_err(PistonError::VoxelRenderer)?;
                debug_renderer.render(&mut win.encoder, &win.output_color, &win.output_stencil, camera_projection)
                    .map_err(PistonError::DebugRendererRender)?;
            }
            Ok(())
        });
        if let Some(result) = maybe_result {
            let () = result.map_err(Error::Piston)?;
        }

        match event {
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::Q), state: ButtonState::Release, .. })) =>
                return Ok(()),
            _ =>
                (),
        }
    }
}
