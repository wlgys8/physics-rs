use std::time::Instant;

use fast_mass_spring::{
    cloth::{Attachment, Cloth, ClothBuilder},
    solver::FastMassSpringSolver,
};
use simulation::{math::Isometry3, FixedFrames, GridPlaneBuilder};
use three_d::{
    egui::{Slider, Widget},
    Camera, ClearState, FrameInput,
};

use crate::{
    common::{ClothOptions, Demo, DemoLoopResult, SolverOptions},
    gui::{ClothOptionsGUI, SolverOptionsGUI},
    render::ClothRender,
};

pub struct HangClothScene {
    solver: FastMassSpringSolver,
    render: ClothRender,
    fixed_frame_generator: FixedFrames,
}

impl HangClothScene {
    fn new(context: &three_d::Context, scene_options: SceneOptions) -> Self {
        let solver_options = scene_options.solver_options;
        let mut render = ClothRender::new(context);
        let (cloth, mesh) = create_cloth(scene_options);
        render.set_indices(mesh.indices());
        render.set_vertices_from_slice(cloth.particle_positions.as_slice());

        let time_step = solver_options.time_step;
        let mut solver: FastMassSpringSolver = FastMassSpringSolver::new(cloth, time_step);
        solver.set_num_iterations(solver_options.num_iterations);
        solver.set_gravity(solver_options.gravity);

        let fixed_frame_generator = FixedFrames::new(time_step);

        Self {
            solver,
            render,
            fixed_frame_generator,
        }
    }

    pub fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        let mut step_count = 0;
        let time = Instant::now();
        for _ in self
            .fixed_frame_generator
            .iter((frame_input.accumulated_time / 1000.0) as f32, 1)
        {
            self.solver.step();
            step_count += 1;
        }

        let result = if step_count > 0 {
            let cost = time.elapsed() / step_count;
            self.render
                .set_vertices_from_slice(self.solver.cloth().particle_positions.as_slice());
            DemoLoopResult {
                updated: true,
                step_cost: cost,
            }
        } else {
            DemoLoopResult::not_updated()
        };
        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
            .write(|| {
                self.render.draw(camera, frame_input.viewport);
            });
        result
    }
}

#[derive(Default)]
pub struct HangClothDemo {
    scene: Option<HangClothScene>,
    scene_options: SceneOptions,
}

impl Demo for HangClothDemo {
    fn name(&self) -> &'static str {
        "HangCloth"
    }

    fn restart(&mut self, context: &three_d::Context) {
        self.scene = Some(HangClothScene::new(context, self.scene_options));
    }

    fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        if let Some(scene) = self.scene.as_mut() {
            scene.on_frame_loop(camera, frame_input)
        } else {
            DemoLoopResult::not_updated()
        }
    }

    fn show_options_gui(&mut self, ui: &mut three_d::egui::Ui) {
        SolverOptionsGUI::new(&mut self.scene_options.solver_options).show_ui(ui);
        ClothOptionsGUI::new(&mut self.scene_options.cloth_options).show_ui(ui);
        Slider::new(&mut self.scene_options.attachment_stiffness, 0.1..=100.0)
            .text("Attachment Stiffness")
            .ui(ui);
        ui.checkbox(&mut self.scene_options.fix_left_top, "Fix Left Top");
        ui.checkbox(&mut self.scene_options.fix_right_top, "Fix Right Top");
    }
}

fn create_cloth(options: SceneOptions) -> (Cloth, simulation::Mesh) {
    let cloth_options = options.cloth_options;
    let resolution = cloth_options.resolution;
    let cloth_size = 3.0;
    let transform = Isometry3::translation(0.0, 0.0, 0.0);

    let physics_cloth_builder: ClothBuilder = ClothBuilder {
        size: cloth_size,
        resolution,
        structural_spring_stiffness: cloth_options.structual_spring_stiffness,
        shear_spring_stiffness: cloth_options.shear_spring_stiffness,
        mass: cloth_options.mass,
        transform,
    };

    let top_left = physics_cloth_builder.top_left_vertex_index();
    let top_right = physics_cloth_builder.top_right_vertex_index();

    let mut cloth = physics_cloth_builder.build();

    if options.fix_left_top {
        cloth.add_attachments([Attachment {
            particle_index: top_left,
            target_position: cloth.get_particle_position(top_left),
            stiffness: options.attachment_stiffness,
        }]);
    }

    if options.fix_right_top {
        cloth.add_attachments([Attachment {
            particle_index: top_right,
            target_position: cloth.get_particle_position(top_right),
            stiffness: options.attachment_stiffness,
        }]);
    }

    let render_mesh_data =
        GridPlaneBuilder::new(cloth_size, cloth_size, resolution - 1, resolution - 1)
            .with_transform(transform)
            .build();
    (cloth, render_mesh_data)
}

#[derive(Clone, Copy)]
struct SceneOptions {
    solver_options: SolverOptions,
    cloth_options: ClothOptions,
    fix_left_top: bool,
    fix_right_top: bool,
    attachment_stiffness: f32,
}

impl Default for SceneOptions {
    fn default() -> Self {
        Self {
            solver_options: SolverOptions::default(),
            cloth_options: ClothOptions {
                structual_spring_stiffness: 100.0,
                shear_spring_stiffness: 0.2,
                ..Default::default()
            },
            fix_left_top: true,
            fix_right_top: true,
            attachment_stiffness: 50.0,
        }
    }
}
