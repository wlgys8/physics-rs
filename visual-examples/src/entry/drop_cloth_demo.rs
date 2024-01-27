use std::time::Instant;

use fast_mass_spring::{
    cloth::{Cloth, ClothFromMeshBuilder},
    solver::FastMassSpringSolver,
};
use simulation::{math::Isometry3, FixedFrameGenerator, GridPlaneBuilder, SphereCollider};
use three_d::{
    AmbientLight, Camera, CpuMaterial, CpuMesh, DirectionalLight, FrameInput, Gm, PhysicalMaterial,
    Srgba,
};

use crate::{
    common::{ClothOptions, Demo, DemoLoopResult, PhysicsOptions},
    gui::ClothOptionsGUI,
    render::ClothRender,
};

pub struct DropClothScene {
    solver: FastMassSpringSolver,
    cloth_render: ClothRender,
    fixed_frame_generator: FixedFrameGenerator,
    sphere_render: Gm<three_d::Mesh, PhysicalMaterial>,
    lights: Lights,
}

impl DropClothScene {
    pub fn new(
        context: &three_d::Context,
        physics_options: PhysicsOptions,
        cloth_options: ClothOptions,
    ) -> Self {
        let mut render = ClothRender::new(context);
        let (cloth, mesh) = create_cloth(cloth_options);
        render.set_indices(mesh.indices());

        let mut solver: FastMassSpringSolver =
            FastMassSpringSolver::new(cloth, physics_options.time_step);
        solver.set_num_iterations(physics_options.num_iterations);
        solver.set_gravity(physics_options.gravity);
        solver.add_collider(
            SphereCollider { radius: 1.0 },
            simulation::math::Isometry3::identity(),
        );

        let fixed_frame_generator = FixedFrameGenerator::new(physics_options.time_step);

        Self {
            solver,
            cloth_render: render,
            fixed_frame_generator,
            sphere_render: create_sphere_render(context),
            lights: Lights::new(context),
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
            let step_cost = time.elapsed() / step_count;
            self.cloth_render
                .set_vertices_from_slice(self.solver.cloth().particle_positions.as_slice());
            DemoLoopResult {
                updated: true,
                step_cost,
            }
        } else {
            DemoLoopResult::not_updated()
        };

        frame_input
            .screen()
            .write(|| {
                self.cloth_render.draw(camera, frame_input.viewport);
            })
            .render(camera, [&self.sphere_render], &self.lights.array());
        result
    }
}

pub struct DropClothDemo {
    scene: Option<DropClothScene>,
    cloth_options: ClothOptionsGUI,
}

impl Default for DropClothDemo {
    fn default() -> Self {
        Self {
            scene: None,
            cloth_options: ClothOptionsGUI {
                data: ClothOptions {
                    spring_stiffness: 0.5,
                },
            },
        }
    }
}

impl Demo for DropClothDemo {
    fn name(&self) -> &'static str {
        "Drop Cloth"
    }

    fn restart(&mut self, context: &three_d::Context, physics_options: PhysicsOptions) {
        self.scene = Some(DropClothScene::new(
            context,
            physics_options,
            self.cloth_options.data,
        ));
    }

    fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        if let Some(scene) = &mut self.scene {
            scene.on_frame_loop(camera, frame_input)
        } else {
            DemoLoopResult::not_updated()
        }
    }

    fn show_options_gui(&mut self, ui: &mut three_d::egui::Ui) {
        self.cloth_options.show_ui(ui);
    }
}

fn create_cloth(options: ClothOptions) -> (Cloth, simulation::Mesh) {
    let grid_builder = GridPlaneBuilder::new(4.0, 4.0, 18, 18).with_transform(Isometry3 {
        rotation: simulation::math::UnitQuaternion::from_axis_angle(
            &simulation::math::Vector3::x_axis(),
            std::f32::consts::PI / 2.0,
        ),
        translation: simulation::math::Vector3::new(0.0, 1.2, 0.0).into(),
    });

    let mesh = grid_builder.build();

    let cloth = ClothFromMeshBuilder {
        mesh: &mesh,
        mass: 0.01,
        spring_stiffness: options.spring_stiffness,
    }
    .build();

    (cloth, mesh)
}

fn create_sphere_render(context: &three_d::Context) -> Gm<three_d::Mesh, PhysicalMaterial> {
    use three_d::Mat4;
    let mut sphere = CpuMesh::sphere(64);
    sphere.transform(&Mat4::from_scale(0.95)).unwrap();
    Gm::new(
        three_d::Mesh::new(context, &sphere),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo: Srgba::BLUE,
                metallic: 1.0,
                ..Default::default()
            },
        ),
    )
}

struct Lights {
    ambient: AmbientLight,
    directional: DirectionalLight,
}

impl Lights {
    pub fn new(context: &three_d::Context) -> Self {
        Self {
            ambient: AmbientLight::new(context, 0.4, Srgba::WHITE),
            directional: DirectionalLight::new(
                context,
                2.0,
                Srgba::WHITE,
                &three_d::vec3(-1.0, -1.0, -1.0),
            ),
        }
    }

    pub fn array(&self) -> [&dyn three_d::Light; 2] {
        [&self.ambient, &self.directional]
    }
}
