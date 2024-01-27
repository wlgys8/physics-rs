use std::time::Instant;

use fast_mass_spring::{
    cloth::{Attachment, Cloth, ClothFromMeshBuilder},
    solver::FastMassSpringSolver,
};
use simulation::{math::Isometry3, FixedFrameGenerator, GridPlaneBuilder};
use three_d::{Camera, ClearState, FrameInput};

use crate::{
    common::{ClothOptions, Demo, DemoLoopResult, PhysicsOptions},
    gui::ClothOptionsGUI,
    render::ClothRender,
};

pub struct HangClothScene {
    solver: FastMassSpringSolver,
    render: ClothRender,
    fixed_frame_generator: FixedFrameGenerator,
}

impl HangClothScene {
    pub fn new(
        context: &three_d::Context,
        physics_options: PhysicsOptions,
        cloth_options: ClothOptions,
    ) -> Self {
        let mut render = ClothRender::new(context);
        let (cloth, mesh) = create_cloth(cloth_options);
        render.set_indices(mesh.indices());
        render.set_vertices_from_slice(cloth.particle_positions.as_slice());

        let time_step = physics_options.time_step;
        let mut solver: FastMassSpringSolver = FastMassSpringSolver::new(cloth, time_step);
        solver.set_num_iterations(physics_options.num_iterations);
        solver.set_gravity(physics_options.gravity);

        let fixed_frame_generator = FixedFrameGenerator::new(time_step);

        Self {
            solver,
            render,
            fixed_frame_generator,
        }
    }

    pub fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        let mut step_count = 0;
        let time = Instant::now();
        while self
            .fixed_frame_generator
            .next((frame_input.accumulated_time / 1000.0) as f32)
        {
            self.solver.step();
            step_count += 1;
            if step_count >= 5 {
                break;
            }
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
    cloth_options: ClothOptionsGUI,
}

impl Demo for HangClothDemo {
    fn name(&self) -> &'static str {
        "HangCloth"
    }

    fn restart(&mut self, context: &three_d::Context, physics_options: PhysicsOptions) {
        self.scene = Some(HangClothScene::new(
            context,
            physics_options,
            self.cloth_options.data,
        ));
    }

    fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        if let Some(scene) = self.scene.as_mut() {
            scene.on_frame_loop(camera, frame_input)
        } else {
            DemoLoopResult::not_updated()
        }
    }

    fn show_options_gui(&mut self, ui: &mut three_d::egui::Ui) {
        self.cloth_options.show_ui(ui);
    }
}

fn create_cloth(cloth: ClothOptions) -> (Cloth, simulation::Mesh) {
    let grid_builder = GridPlaneBuilder::new(3.0, 3.0, 10, 10)
        .with_transform(Isometry3::translation(1.0, 0.0, 0.0));

    let top_left = grid_builder.top_left_vertex_index();
    let mesh = grid_builder.build();

    let mut cloth = ClothFromMeshBuilder {
        mesh: &mesh,
        mass: 0.01,
        spring_stiffness: cloth.spring_stiffness,
    }
    .build();

    cloth.add_attachments([Attachment {
        particle_index: top_left,
        target_position: mesh.vertices()[top_left],
        stiffness: 1.0,
    }]);
    (cloth, mesh)
}
