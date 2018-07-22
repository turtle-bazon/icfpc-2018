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
    fs,
    io::{self, Write},
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

use icfpc2018_lib::{
    model,
    coord::{
        Coord,
        Matrix,
        Resolution,
    },
    cmd::{self, BotCommand},
    router,
};

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
             .default_value("../../problems/FA001_tgt.mdl")
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
        DebugRenderer::new(window.factory.clone(), text_renderer, 65536)
            .map_err(PistonError::DebugRendererInit)
            .map_err(Error::Piston)?
    };
    let mut voxel_renderer = voxel::VoxelRenderer::new(window.factory.clone(), 65536)
        .map_err(PistonError::VoxelRenderer)
        .map_err(Error::Piston)?;

    let model = mat4_id();
    let mut projection = CameraPerspective {
        fov: 90.0f32,
        near_clip: 0.1,
        far_clip: 1000.0,
        aspect_ratio: (SCREEN_WIDTH as f32) / (SCREEN_HEIGHT as f32)
    }.projection();

    let center_point_value = (matrix.dim() / 2) as f32;
    let mut orbit_zoom_camera: OrbitZoomCamera<f32> = OrbitZoomCamera::new(
        [center_point_value, center_point_value, -center_point_value],
        OrbitZoomCameraSettings::default()
    );
    orbit_zoom_camera.distance = (matrix.dim() * 5 / 4) as f32;

    enum CursorState {
        Moving,
        Filling,
    }

    let mut script = Vec::new();
    let mut filled_matrix = Matrix::new(Resolution(matrix.dim() as isize));
    let mut nanobot = Coord { x: 0, y: 0, z: 0, };
    let mut cursor = Coord { x: 1, y: 0, z: 1, };
    let mut cursor_state = CursorState::Moving;
    let mut last_route: Option<Vec<Coord>> = None;
    let mut show_model = true;

    loop {
        let event = if let Some(ev) = window.next() {
            ev
        } else {
            return Ok(());
        };

        event.resize(|width, height| {
            // Update projection matrix
            projection = CameraPerspective {
                fov: 90.0f32,
                near_clip: 0.1,
                far_clip: 1000.0,
                aspect_ratio: (width as f32) / (height as f32)
            }.projection();
        });
        orbit_zoom_camera.event(&event);

        let maybe_result = window.draw_3d(&event, |win| {
            if let Some(args) = event.render_args() {
                win.encoder.clear(&win.output_color, [1.0, 1.0, 1.0, 1.0]);
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
                debug_renderer.draw_line([0.0, 0.0, 0.0], [0.0, 0.0, -5.0], [0.0, 0.0, 1.0, 1.0]);

                debug_renderer.draw_text_at_position("X", [6.0, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0]);
                debug_renderer.draw_text_at_position("Y", [0.0, 6.0, 0.0], [0.0, 1.0, 0.0, 1.0]);
                debug_renderer.draw_text_at_position("Z", [0.0, 0.0, -6.0], [0.0, 0.0, 1.0, 1.0]);

                {
                    let dim = matrix.dim() as f32;

                    // Draw floor
                    //voxel_renderer.draw_voxel([0.0, 0.0, 0.0], [dim, -0.5, dim], [0.33, 0.33, 0.33, 0.5]);
                    for i in 0 .. matrix.dim() {
                        debug_renderer.draw_line([i as f32, 0.0, 0.0], [i as f32, 0.0, -dim], [0.0, 0.0, 0.0, 1.0]);
                        debug_renderer.draw_line([0.0, 0.0, - (i as f32)], [dim, 0.0, -(i as f32)], [0.0, 0.0, 0.0, 1.0]);
                    }

                    // Draw last route
                    if let Some(route) = last_route.as_ref() {
                        for mark in route.iter() {
                            voxel_renderer.draw_voxel(
                                [mark.x as f32 + 0.3, mark.y as f32 + 0.3, mark.z as f32 + 0.3],
                                [mark.x as f32 + 0.7, mark.y as f32 + 0.7, mark.z as f32 + 0.7],
                                [0.0, 0.5, 0.0, 1.0],
                            );
                        }
                    }

                    let mut draw_cube_mesh = |min: [f32; 3], max: [f32; 3], color| {
                        // front
                        debug_renderer.draw_line([min[0], min[1], -min[2]], [max[0], min[1], -min[2]], color);
                        debug_renderer.draw_line([max[0], min[1], -min[2]], [max[0], max[1], -min[2]], color);
                        debug_renderer.draw_line([max[0], max[1], -min[2]], [min[0], max[1], -min[2]], color);
                        debug_renderer.draw_line([min[0], max[1], -min[2]], [min[0], min[1], -min[2]], color);
                        // back
                        debug_renderer.draw_line([min[0], min[1], -max[2]], [max[0], min[1], -max[2]], color);
                        debug_renderer.draw_line([max[0], min[1], -max[2]], [max[0], max[1], -max[2]], color);
                        debug_renderer.draw_line([max[0], max[1], -max[2]], [min[0], max[1], -max[2]], color);
                        debug_renderer.draw_line([min[0], max[1], -max[2]], [min[0], min[1], -max[2]], color);
                        // missing edges
                        debug_renderer.draw_line([min[0], min[1], -min[2]], [min[0], min[1], -max[2]], color);
                        debug_renderer.draw_line([max[0], min[1], -min[2]], [max[0], min[1], -max[2]], color);
                        debug_renderer.draw_line([max[0], max[1], -min[2]], [max[0], max[1], -max[2]], color);
                        debug_renderer.draw_line([min[0], max[1], -min[2]], [min[0], max[1], -max[2]], color);
                    };

                    // Draw bounding volume
                    draw_cube_mesh([0.0, 0.0, 0.0], [dim, dim, dim], [0.0, 0.0, 0.0, 1.0]);

                    // Draw model matrix
                    if show_model {
                        for voxel in matrix.filled_voxels() {
                            if filled_matrix.is_filled(&voxel) {
                                continue;
                            }
                            // draw voxel
                            let min_point = [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                            let max_point = vec3_add(min_point, [1.0, 1.0, 1.0]);
                            voxel_renderer.draw_voxel(min_point, max_point, [0.0, 0.0, 0.0, 0.15]);
                            // draw mesh
                            let position =
                                [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                            draw_cube_mesh(position, vec3_add(position, [1.0, 1.0, 1.0]), [0.0, 0.0, 0.0, 1.0]);
                        }
                    }

                    // Draw filled matrix
                    for voxel in filled_matrix.filled_voxels() {
                        // draw voxel
                        let min_point = [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                        let max_point = vec3_add(min_point, [1.0, 1.0, 1.0]);
                        voxel_renderer.draw_voxel(min_point, max_point, [0.54, 0.27, 0.07, 1.0]);
                        // draw mesh
                        let position =
                            [voxel.x as f32, voxel.y as f32, voxel.z as f32];
                        draw_cube_mesh(position, vec3_add(position, [1.0, 1.0, 1.0]), [0.0, 0.0, 0.0, 1.0]);
                    }

                    // Draw cursor
                    let cursor_color = match cursor_state {
                        CursorState::Moving =>
                            if filled_matrix.is_filled(&cursor) {
                                [1.0, 0.0, 0.0, 1.0]
                            } else {
                                [0.0, 0.0, 1.0, 1.0]
                            },
                        CursorState::Filling =>
                            if cursor.diff(&nanobot).is_near() && !filled_matrix.is_filled(&cursor) {
                                [0.0, 1.0, 0.0, 1.0]
                            } else {
                                [1.0, 0.0, 0.0, 1.0]
                            },
                    };
                    let min_point = [cursor.x as f32, cursor.y as f32, cursor.z as f32];
                    let max_point = vec3_add(min_point, [1.0, 1.0, 1.0]);
                    voxel_renderer.draw_voxel(min_point, max_point, [cursor_color[0], cursor_color[1], cursor_color[2], 0.5]);
                    draw_cube_mesh(min_point, vec3_add(min_point, [1.0, 1.0, 1.0]), cursor_color);
                    let x_proj_min = [0.0, min_point[1], min_point[2]];
                    let x_proj_max = vec3_add(x_proj_min, [0.0, 1.0, 1.0]);
                    voxel_renderer.draw_voxel(x_proj_min, x_proj_max, [cursor_color[0], cursor_color[1], cursor_color[2], 0.5]);
                    let y_proj_min = [min_point[0], 0.0, min_point[2]];
                    let y_proj_max = vec3_add(y_proj_min, [1.0, 0.0, 1.0]);
                    voxel_renderer.draw_voxel(y_proj_min, y_proj_max, [cursor_color[0], cursor_color[1], cursor_color[2], 0.5]);
                    let z_proj_min = [min_point[0], min_point[1], 0.0];
                    let z_proj_max = vec3_add(z_proj_min, [1.0, 1.0, 0.0]);
                    voxel_renderer.draw_voxel(z_proj_min, z_proj_max, [cursor_color[0], cursor_color[1], cursor_color[2], 0.5]);

                    // Draw nanobot
                    let min_point = [nanobot.x as f32, nanobot.y as f32, nanobot.z as f32];
                    let max_point = vec3_add(min_point, [1.0, 1.0, 1.0]);
                    voxel_renderer.draw_voxel(min_point, max_point, [1.0, 1.0, 0.0, 1.0]);
                    draw_cube_mesh(min_point, vec3_add(min_point, [1.0, 1.0, 1.0]), [0.0, 0.0, 0.0, 1.0]);
                }

                let total = script.len();
                let mut oi = 0;
                for (i, cmd) in script.iter().enumerate() {
                    if (i as isize) < (total as isize) - 10 {
                        continue;
                    }

                    debug_renderer.draw_text_on_screen(
                        &format!("{}: {:?}", i, cmd),
                        [10, 10 + oi * 20],
                        [0.0, 0.0, 0.0, 1.0],
                    );
                    oi += 1;
                }

                voxel_renderer.render(&mut win.encoder, &win.output_color, &win.output_stencil, camera_projection)
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
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::I), state: ButtonState::Release, .. })) =>
                show_model = !show_model,
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::A), state: ButtonState::Release, .. })) =>
                if cursor.x > 0 { cursor.x -= 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::S), state: ButtonState::Release, .. })) =>
                if cursor.z > 0 { cursor.z -= 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::D), state: ButtonState::Release, .. })) =>
                if cursor.x + 1 < matrix.dim() as isize { cursor.x += 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::W), state: ButtonState::Release, .. })) =>
                if cursor.z + 1 < matrix.dim() as isize { cursor.z += 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::F), state: ButtonState::Release, .. })) =>
                if cursor.y > 0 { cursor.y -= 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::R), state: ButtonState::Release, .. })) =>
                if cursor.y + 1 < matrix.dim() as isize { cursor.y += 1; },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::Tab), state: ButtonState::Release, .. })) =>
                match cursor_state {
                    CursorState::Moving =>
                        cursor_state = CursorState::Filling,
                    CursorState::Filling =>
                        cursor_state = CursorState::Moving,
                },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::Space), state: ButtonState::Release, .. })) =>
                match cursor_state {
                    CursorState::Moving =>
                        if !filled_matrix.is_filled(&cursor) {
                            let maybe_route = router::rtt::plan_route(
                                &nanobot,
                                &cursor,
                                &filled_matrix,
                                None.into_iter(),
                                matrix.dim() * matrix.dim() * matrix.dim(),
                            );
                            if let Some(route) = maybe_route {
                                let mut script_route = Vec::new();
                                router::rtt::plan_route_commands(&route, &mut script_route);
                                if !script_route.is_empty() {
                                    last_route = Some(script_route.iter().map(|mv| mv.0).collect());
                                    script.extend(script_route.into_iter().map(|mv| mv.1));
                                    nanobot = cursor;
                                }
                            }
                        },
                    CursorState::Filling =>
                        if nanobot.diff(&cursor).is_near() && !filled_matrix.is_filled(&cursor) {
                            filled_matrix.set_filled(&cursor);
                            script.push(BotCommand::fill(nanobot.diff(&cursor)).unwrap());
                        },
                },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::P), state: ButtonState::Release, .. })) =>
                if let CursorState::Filling = cursor_state {
                    for coord in nanobot.get_neighbours() {
                        if coord.y >= nanobot.y {
                            continue;
                        }
                        if filled_matrix.is_filled(&coord) {
                            continue;
                        }
                        if !matrix.is_filled(&coord) {
                            continue;
                        }
                        filled_matrix.set_filled(&coord);
                        script.push(BotCommand::fill(nanobot.diff(&coord)).unwrap());
                    }
                },
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::H), state: ButtonState::Release, .. })) =>
                if nanobot.x == 0 && nanobot.y == 0 && nanobot.z == 0 {
                    script.push(BotCommand::halt().unwrap());
                    let trace = cmd::into_bytes(&script).unwrap();
                    let file = fs::File::create("a.nbt").unwrap();
                    let mut writer = io::BufWriter::new(file);
                    writer.write_all(&trace).unwrap();
                },
            _ =>
                (),
        }
    }
}
