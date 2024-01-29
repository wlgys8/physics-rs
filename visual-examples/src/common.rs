use simulation::math::Vector3;
use three_d::{Camera, FrameInput};

pub trait Demo {
    fn name(&self) -> &'static str;
    fn restart(&mut self, context: &three_d::Context);
    fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult;
    fn show_options_gui(&mut self, ui: &mut three_d::egui::Ui);
}

pub struct DemoLoopResult {
    pub updated: bool,
    pub step_cost: std::time::Duration,
}

impl DemoLoopResult {
    pub fn not_updated() -> Self {
        Self {
            updated: false,
            step_cost: std::time::Duration::default(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct SolverOptions {
    pub time_step: f32,
    pub gravity: Vector3,
    pub num_iterations: usize,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            time_step: 1.0 / 60.0,
            gravity: Vector3::new(0.0, -9.8, 0.0),
            num_iterations: 2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ClothOptions {
    pub structual_spring_stiffness: f32,
    pub shear_spring_stiffness: f32,
    pub mass: f32,
    pub resolution: usize,
}

impl Default for ClothOptions {
    fn default() -> Self {
        Self {
            structual_spring_stiffness: 10.0,
            shear_spring_stiffness: 0.6,
            mass: 1.0,
            resolution: 20,
        }
    }
}
