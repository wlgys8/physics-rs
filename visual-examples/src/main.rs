use entry::DemoEntry;
use three_d::*;
mod common;
mod entry;
mod gui;
mod render;

pub fn main() {
    // Create a window (a canvas on web)
    let window = Window::new(WindowSettings {
        title: "Simulation!".to_string(),
        #[cfg(not(target_arch = "wasm32"))]
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();

    // Get the graphics context from the window
    let context: Context = window.gl();

    let mut camera: Camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.0, 0.0, 10.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.1,
        100.0,
    );
    let mut control = OrbitControl::new(*camera.target(), 0.5, 100.0);
    let mut demo_entry = DemoEntry::new(&context);
    // Start the main render loop
    window.render_loop(
        move |mut frame_input| // Begin a new frame with an updated frame input
    {
        // Ensure the viewport matches the current window viewport which changes if the window is resized
         camera.set_viewport(frame_input.viewport);
         control.handle_events(&mut camera, &mut frame_input.events);
            frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0));
         demo_entry.render_loop(&context,&camera,&mut frame_input);

        FrameOutput::default()
    },
    );
}
